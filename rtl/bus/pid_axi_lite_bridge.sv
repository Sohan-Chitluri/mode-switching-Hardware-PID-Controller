//==============================================================================
// Module: pid_axi_lite_bridge.sv
// Description: Synthesizable AMBA AXI4-Lite Slave Protocol Bridge for pid_soc_top.
//              Converts standard AXI4-Lite read/write channel handshakes into
//              internal synchronous MMIO accesses for the Hardware PID Flight Controller.
//
// Compliant with ARM AMBA AXI Protocol Specification v2.0 (AXI4-Lite).
//==============================================================================

`ifndef PID_AXI_LITE_BRIDGE_SV
`define PID_AXI_LITE_BRIDGE_SV

module pid_axi_lite_bridge #(
    parameter int ADDR_W = 11, // Byte address width (word address is [ADDR_W-1:2])
    parameter int DATA_W = 32
) (
    // AXI Clock & Reset
    input  logic                     aclk,
    input  logic                     aresetn,

    // AXI4-Lite Write Address Channel
    input  logic [ADDR_W-1:0]        s_axi_awaddr,
    input  logic [2:0]               s_axi_awprot,
    input  logic                     s_axi_awvalid,
    output logic                     s_axi_awready,

    // AXI4-Lite Write Data Channel
    input  logic [DATA_W-1:0]        s_axi_wdata,
    input  logic [(DATA_W/8)-1:0]    s_axi_wstrb,
    input  logic                     s_axi_wvalid,
    output logic                     s_axi_wready,

    // AXI4-Lite Write Response Channel
    output logic [1:0]               s_axi_bresp,
    output logic                     s_axi_bvalid,
    input  logic                     s_axi_bready,

    // AXI4-Lite Read Address Channel
    input  logic [ADDR_W-1:0]        s_axi_araddr,
    input  logic [2:0]               s_axi_arprot,
    input  logic                     s_axi_arvalid,
    output logic                     s_axi_arready,

    // AXI4-Lite Read Data Channel
    output logic [DATA_W-1:0]        s_axi_rdata,
    output logic [1:0]               s_axi_rresp,
    output logic                     s_axi_rvalid,
    input  logic                     s_axi_rready,

    // Internal Synchronous MMIO Bus to pid_soc_top
    output logic                     bus_we,
    output logic [8:0]               bus_addr,
    output logic signed [DATA_W-1:0] bus_wdata,
    input  logic signed [DATA_W-1:0] bus_rdata,
    input  logic                     bus_rvalid
);

    localparam [1:0] RESP_OKAY   = 2'b00;
    localparam [1:0] RESP_SLVERR = 2'b10;

    // -------------------------------------------------------------------------
    // Write Channel Handshake & FSM
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        WR_IDLE = 2'b00,
        WR_EXEC = 2'b01,
        WR_RESP = 2'b10
    } wr_state_t;

    wr_state_t wr_state;
    logic [8:0] wr_addr_latched;
    logic [DATA_W-1:0] wr_data_latched;

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            wr_state        <= WR_IDLE;
            s_axi_awready   <= 1'b0;
            s_axi_wready    <= 1'b0;
            s_axi_bvalid    <= 1'b0;
            s_axi_bresp     <= RESP_OKAY;
            wr_addr_latched <= 9'd0;
            wr_data_latched <= '0;
        end else begin
            case (wr_state)
                WR_IDLE: begin
                    s_axi_bvalid <= 1'b0;
                    if (s_axi_awvalid && s_axi_wvalid) begin
                        s_axi_awready   <= 1'b1;
                        s_axi_wready    <= 1'b1;
                        wr_addr_latched <= s_axi_awaddr[10:2];
                        wr_data_latched <= s_axi_wdata;
                        wr_state        <= WR_EXEC;
                    end else begin
                        s_axi_awready   <= 1'b0;
                        s_axi_wready    <= 1'b0;
                    end
                end

                WR_EXEC: begin
                    s_axi_awready <= 1'b0;
                    s_axi_wready  <= 1'b0;
                    s_axi_bvalid  <= 1'b1;
                    s_axi_bresp   <= RESP_OKAY;
                    wr_state      <= WR_RESP;
                end

                WR_RESP: begin
                    if (s_axi_bready) begin
                        s_axi_bvalid <= 1'b0;
                        wr_state     <= WR_IDLE;
                    end
                end

                default: wr_state <= WR_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Read Channel Handshake & FSM
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        RD_IDLE = 2'b00,
        RD_WAIT = 2'b01,
        RD_RESP = 2'b10
    } rd_state_t;

    rd_state_t rd_state;
    logic [8:0] rd_addr_latched;

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            rd_state        <= RD_IDLE;
            s_axi_arready   <= 1'b0;
            s_axi_rvalid    <= 1'b0;
            s_axi_rdata     <= '0;
            s_axi_rresp     <= RESP_OKAY;
            rd_addr_latched <= 9'd0;
        end else begin
            case (rd_state)
                RD_IDLE: begin
                    s_axi_rvalid <= 1'b0;
                    if (s_axi_arvalid && wr_state != WR_EXEC) begin
                        s_axi_arready   <= 1'b1;
                        rd_addr_latched <= s_axi_araddr[10:2];
                        rd_state        <= RD_WAIT;
                    end else begin
                        s_axi_arready   <= 1'b0;
                    end
                end

                RD_WAIT: begin
                    s_axi_arready <= 1'b0;
                    if (bus_rvalid) begin
                        s_axi_rvalid <= 1'b1;
                        s_axi_rdata  <= bus_rdata;
                        s_axi_rresp  <= RESP_OKAY;
                        rd_state     <= RD_RESP;
                    end
                end

                RD_RESP: begin
                    if (s_axi_rready) begin
                        s_axi_rvalid <= 1'b0;
                        rd_state     <= RD_IDLE;
                    end
                end

                default: rd_state <= RD_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Internal Bus Arbitrated Multiplexing
    // -------------------------------------------------------------------------
    always_comb begin
        if (wr_state == WR_EXEC) begin
            bus_we    = 1'b1;
            bus_addr  = wr_addr_latched;
            bus_wdata = wr_data_latched;
        end else if (rd_state == RD_WAIT) begin
            bus_we    = 1'b0;
            bus_addr  = rd_addr_latched;
            bus_wdata = '0;
        end else if (s_axi_arvalid && wr_state == WR_IDLE) begin
            bus_we    = 1'b0;
            bus_addr  = s_axi_araddr[10:2];
            bus_wdata = '0;
        end else begin
            bus_we    = 1'b0;
            bus_addr  = 9'd0;
            bus_wdata = '0;
        end
    end

endmodule : pid_axi_lite_bridge

`endif // PID_AXI_LITE_BRIDGE_SV

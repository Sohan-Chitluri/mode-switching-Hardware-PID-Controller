//==============================================================================
// Module: pid_apb4_bridge.sv
// Description: Synthesizable AMBA APB4 Slave Protocol Bridge for pid_soc_top.
//              Converts standard APB4 transfers into internal synchronous MMIO
//              accesses for the Hardware PID Flight Controller.
//
// Compliant with ARM AMBA APB Protocol Specification v2.0 (APB4).
//==============================================================================

`ifndef PID_APB4_BRIDGE_SV
`define PID_APB4_BRIDGE_SV

module pid_apb4_bridge #(
    parameter int ADDR_W = 11, // Byte address width (word address is [ADDR_W-1:2])
    parameter int DATA_W = 32
) (
    // APB4 Clock & Reset
    input  logic                     pclk,
    input  logic                     presetn,

    // APB4 Slave Interface
    input  logic [ADDR_W-1:0]        paddr,
    input  logic                     psel,
    input  logic                     penable,
    input  logic                     pwrite,
    input  logic [DATA_W-1:0]        pwdata,
    input  logic [(DATA_W/8)-1:0]    pstrb,
    input  logic [2:0]               pprot,
    output logic [DATA_W-1:0]        prdata,
    output logic                     pready,
    output logic                     pslverr,

    // Internal Synchronous MMIO Bus to pid_soc_top
    output logic                     bus_we,
    output logic [8:0]               bus_addr,
    output logic signed [DATA_W-1:0] bus_wdata,
    input  logic signed [DATA_W-1:0] bus_rdata,
    input  logic                     bus_rvalid
);

    // Byte address to 32-bit word address conversion
    wire [8:0] word_addr = paddr[10:2];

    typedef enum logic [1:0] {
        ST_IDLE   = 2'b00,
        ST_SETUP  = 2'b01,
        ST_ACCESS = 2'b10
    } apb_state_t;

    apb_state_t state, next_state;

    // State register
    always_ff @(posedge pclk) begin
        if (!presetn)
            state <= ST_IDLE;
        else
            state <= next_state;
    end

    // Next-state logic
    always_comb begin
        next_state = state;
        case (state)
            ST_IDLE: begin
                if (psel && !penable)
                    next_state = ST_ACCESS;
            end
            ST_ACCESS: begin
                if (penable && pready)
                    next_state = (psel && !penable) ? ST_ACCESS : ST_IDLE;
            end
            default: next_state = ST_IDLE;
        endcase
    end

    // APB response and bus drive logic
    always_comb begin
        bus_we     = 1'b0;
        bus_addr   = word_addr;
        bus_wdata  = pwdata;
        pready     = 1'b0;
        pslverr    = 1'b0;
        prdata     = bus_rdata;

        if (psel) begin
            if (pwrite) begin
                // Write transaction: active during setup/access
                bus_we = 1'b1;
                if (penable) begin
                    pready = 1'b1; // Single-cycle write completion
                end
            end else begin
                // Read transaction: ready when internal bus_rvalid arrives
                bus_we = 1'b0;
                if (penable) begin
                    pready = bus_rvalid;
                end
            end
        end
    end

endmodule : pid_apb4_bridge

`endif // PID_APB4_BRIDGE_SV

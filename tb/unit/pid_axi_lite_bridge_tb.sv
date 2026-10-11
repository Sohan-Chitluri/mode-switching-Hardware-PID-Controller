//==============================================================================
// Testbench: pid_axi_lite_bridge_tb.sv
// Description: Self-checking unit testbench for AMBA AXI4-Lite Protocol Bridge.
//              Verifies write transactions, read transactions, back-to-back cycles,
//              and OKAY response handshakes with simulated MMIO slave.
//==============================================================================

`timescale 1ns/1ps

`include "pid_axi_lite_bridge.sv"

module pid_axi_lite_bridge_tb;

    localparam int ADDR_W = 11;
    localparam int DATA_W = 32;

    logic aclk;
    logic aresetn;

    // AXI4-Lite Interface
    logic [ADDR_W-1:0]      s_axi_awaddr;
    logic [2:0]             s_axi_awprot;
    logic                   s_axi_awvalid;
    logic                   s_axi_awready;

    logic [DATA_W-1:0]      s_axi_wdata;
    logic [(DATA_W/8)-1:0]  s_axi_wstrb;
    logic                   s_axi_wvalid;
    logic                   s_axi_wready;

    logic [1:0]             s_axi_bresp;
    logic                   s_axi_bvalid;
    logic                   s_axi_bready;

    logic [ADDR_W-1:0]      s_axi_araddr;
    logic [2:0]             s_axi_arprot;
    logic                   s_axi_arvalid;
    logic                   s_axi_arready;

    logic [DATA_W-1:0]      s_axi_rdata;
    logic [1:0]             s_axi_rresp;
    logic                   s_axi_rvalid;
    logic                   s_axi_rready;

    // Simulated internal MMIO bus
    logic                     bus_we;
    logic [8:0]               bus_addr;
    logic signed [DATA_W-1:0] bus_wdata;
    logic signed [DATA_W-1:0] bus_rdata;
    logic                     bus_rvalid;

    // DUT
    pid_axi_lite_bridge #(
        .ADDR_W(ADDR_W),
        .DATA_W(DATA_W)
    ) dut (
        .aclk(aclk),
        .aresetn(aresetn),
        .s_axi_awaddr(s_axi_awaddr),
        .s_axi_awprot(s_axi_awprot),
        .s_axi_awvalid(s_axi_awvalid),
        .s_axi_awready(s_axi_awready),
        .s_axi_wdata(s_axi_wdata),
        .s_axi_wstrb(s_axi_wstrb),
        .s_axi_wvalid(s_axi_wvalid),
        .s_axi_wready(s_axi_wready),
        .s_axi_bresp(s_axi_bresp),
        .s_axi_bvalid(s_axi_bvalid),
        .s_axi_bready(s_axi_bready),
        .s_axi_araddr(s_axi_araddr),
        .s_axi_arprot(s_axi_arprot),
        .s_axi_arvalid(s_axi_arvalid),
        .s_axi_arready(s_axi_arready),
        .s_axi_rdata(s_axi_rdata),
        .s_axi_rresp(s_axi_rresp),
        .s_axi_rvalid(s_axi_rvalid),
        .s_axi_rready(s_axi_rready),
        .bus_we(bus_we),
        .bus_addr(bus_addr),
        .bus_wdata(bus_wdata),
        .bus_rdata(bus_rdata),
        .bus_rvalid(bus_rvalid)
    );

    // Mock internal register memory (64 words)
    logic [31:0] mock_mem [0:63];

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            for (int i = 0; i < 64; i++) mock_mem[i] <= 32'd0;
            bus_rvalid <= 1'b0;
            bus_rdata  <= 32'd0;
        end else begin
            if (bus_we) begin
                if (bus_addr < 9'd64) mock_mem[bus_addr[5:0]] <= bus_wdata;
            end
            // Synchronous 1-cycle read latency
            bus_rvalid <= !bus_we;
            if (bus_addr < 9'd64)
                bus_rdata <= mock_mem[bus_addr[5:0]];
            else
                bus_rdata <= 32'hDEAD_BEEF;
        end
    end

    // Clock generator (50 MHz)
    initial aclk = 0;
    always #10 aclk = ~aclk;

    int error_count = 0;

    // AXI Write Task
    task automatic axi_write(input [ADDR_W-1:0] addr, input [DATA_W-1:0] data);
        @(posedge aclk);
        s_axi_awaddr  <= addr;
        s_axi_awprot  <= 3'b000;
        s_axi_awvalid <= 1'b1;
        s_axi_wdata   <= data;
        s_axi_wstrb   <= 4'b1111;
        s_axi_wvalid  <= 1'b1;
        s_axi_bready  <= 1'b1;

        // Wait for awready and wready
        while (!(s_axi_awready && s_axi_wready)) @(posedge aclk);
        @(posedge aclk);
        s_axi_awvalid <= 1'b0;
        s_axi_wvalid  <= 1'b0;

        // Wait for bvalid
        while (!s_axi_bvalid) @(posedge aclk);
        if (s_axi_bresp != 2'b00) begin
            $display("[FAIL] Write response error at addr 0x%03h, bresp=%b", addr, s_axi_bresp);
            error_count++;
        end
        @(posedge aclk);
        s_axi_bready <= 1'b0;
    endtask

    // AXI Read Task
    task automatic axi_read(input [ADDR_W-1:0] addr, output [DATA_W-1:0] data);
        @(posedge aclk);
        s_axi_araddr  <= addr;
        s_axi_arprot  <= 3'b000;
        s_axi_arvalid <= 1'b1;
        s_axi_rready  <= 1'b1;

        // Wait for arready
        while (!s_axi_arready) @(posedge aclk);
        @(posedge aclk);
        s_axi_arvalid <= 1'b0;

        // Wait for rvalid
        while (!s_axi_rvalid) @(posedge aclk);
        data = s_axi_rdata;
        if (s_axi_rresp != 2'b00) begin
            $display("[FAIL] Read response error at addr 0x%03h, rresp=%b", addr, s_axi_rresp);
            error_count++;
        end
        @(posedge aclk);
        s_axi_rready <= 1'b0;
    endtask

    logic [31:0] read_val;

    initial begin
        $display("==================================================");
        $display("   TESTBENCH: AXI4-Lite Protocol Bridge (Unit)   ");
        $display("==================================================");

        // Reset
        aresetn       = 0;
        s_axi_awaddr  = 0;
        s_axi_awprot  = 0;
        s_axi_awvalid = 0;
        s_axi_wdata   = 0;
        s_axi_wstrb   = 0;
        s_axi_wvalid  = 0;
        s_axi_bready  = 0;
        s_axi_araddr  = 0;
        s_axi_arprot  = 0;
        s_axi_arvalid = 0;
        s_axi_rready  = 0;

        #40;
        aresetn = 1;
        #20;

        // Test 1: Single Register Write and Readback
        $display("[TEST 1] Single write and readback at word 0x03 (byte 0x0C)...");
        axi_write(11'h00C, 32'h0001_8000); // Q16.16 1.5

        axi_read(11'h00C, read_val);
        if (read_val !== 32'h0001_8000) begin
            $display("[FAIL] Readback mismatch! Expected: 0x00018000, Got: 0x%08h", read_val);
            error_count++;
        end else begin
            $display("[PASS] Write and readback verified: 0x%08h", read_val);
        end

        // Test 2: Sequential Multiple Register Accesses
        $display("[TEST 2] Multi-register burst sequence (words 0x04..0x08)...");
        begin
            logic [ADDR_W-1:0] baddr;
            logic [31:0] pattern;
            for (int w = 4; w <= 8; w++) begin
                baddr   = ADDR_W'(w * 4);
                pattern = 32'hA000_0000 | (32'(w) << 16) | (32'(w) * 17);
                axi_write(baddr, pattern);
                axi_read(baddr, read_val);
                if (read_val !== pattern) begin
                    $display("[FAIL] Word 0x%02h mismatch! Expected: 0x%08h, Got: 0x%08h", w, pattern, read_val);
                    error_count++;
                end
            end
        end

        // Summary
        #40;
        if (error_count == 0) begin
            $display("==================================================");
            $display("   [PASS] ALL AXI4-LITE BRIDGE TESTS PASSED      ");
            $display("==================================================");
            $finish(0);
        end else begin
            $display("==================================================");
            $display("   [FAIL] %0d ERRORS DETECTED IN AXI4-LITE TB    ", error_count);
            $display("==================================================");
            $fatal(1);
        end
    end

endmodule : pid_axi_lite_bridge_tb

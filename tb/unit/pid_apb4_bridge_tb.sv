//==============================================================================
// Testbench: pid_apb4_bridge_tb.sv
// Description: Self-checking unit testbench for AMBA APB4 Protocol Bridge.
//              Verifies setup/access phase handshakes, single-cycle write cycles,
//              synchronous read cycles with PREADY, and mock MMIO data integrity.
//==============================================================================

`timescale 1ns/1ps

`include "pid_apb4_bridge.sv"

module pid_apb4_bridge_tb;

    localparam int ADDR_W = 11;
    localparam int DATA_W = 32;

    logic pclk;
    logic presetn;

    // APB4 Interface
    logic [ADDR_W-1:0]        paddr;
    logic                     psel;
    logic                     penable;
    logic                     pwrite;
    logic [DATA_W-1:0]        pwdata;
    logic [(DATA_W/8)-1:0]    pstrb;
    logic [2:0]               pprot;
    logic [DATA_W-1:0]        prdata;
    logic                     pready;
    logic                     pslverr;

    // Internal Synchronous MMIO Bus
    logic                     bus_we;
    logic [8:0]               bus_addr;
    logic signed [DATA_W-1:0] bus_wdata;
    logic signed [DATA_W-1:0] bus_rdata;
    logic                     bus_rvalid;

    // DUT
    pid_apb4_bridge #(
        .ADDR_W(ADDR_W),
        .DATA_W(DATA_W)
    ) dut (
        .pclk(pclk),
        .presetn(presetn),
        .paddr(paddr),
        .psel(psel),
        .penable(penable),
        .pwrite(pwrite),
        .pwdata(pwdata),
        .pstrb(pstrb),
        .pprot(pprot),
        .prdata(prdata),
        .pready(pready),
        .pslverr(pslverr),
        .bus_we(bus_we),
        .bus_addr(bus_addr),
        .bus_wdata(bus_wdata),
        .bus_rdata(bus_rdata),
        .bus_rvalid(bus_rvalid)
    );

    // Mock internal register memory (64 words)
    logic [31:0] mock_mem [0:63];

    always_ff @(posedge pclk) begin
        if (!presetn) begin
            for (int i = 0; i < 64; i++) mock_mem[i] <= 32'd0;
            bus_rvalid <= 1'b0;
            bus_rdata  <= 32'd0;
        end else begin
            if (bus_we) begin
                if (bus_addr < 9'd64) mock_mem[bus_addr[5:0]] <= bus_wdata;
            end
            bus_rvalid <= !bus_we;
            if (bus_addr < 9'd64)
                bus_rdata <= mock_mem[bus_addr[5:0]];
            else
                bus_rdata <= 32'hDEAD_BEEF;
        end
    end

    // Clock generator (50 MHz)
    initial pclk = 0;
    always #10 pclk = ~pclk;

    int error_count = 0;

    // APB Write Task
    task automatic apb_write(input [ADDR_W-1:0] addr, input [DATA_W-1:0] data);
        @(posedge pclk);
        // Setup Phase
        paddr   <= addr;
        pwrite  <= 1'b1;
        psel    <= 1'b1;
        penable <= 1'b0;
        pwdata  <= data;
        pstrb   <= 4'b1111;
        pprot   <= 3'b000;

        @(posedge pclk);
        // Access Phase
        penable <= 1'b1;

        while (!pready) @(posedge pclk);
        @(posedge pclk);
        psel    <= 1'b0;
        penable <= 1'b0;
        pwrite  <= 1'b0;
    endtask

    // APB Read Task
    task automatic apb_read(input [ADDR_W-1:0] addr, output [DATA_W-1:0] data);
        @(posedge pclk);
        // Setup Phase
        paddr   <= addr;
        pwrite  <= 1'b0;
        psel    <= 1'b1;
        penable <= 1'b0;

        @(posedge pclk);
        // Access Phase
        penable <= 1'b1;

        while (!pready) @(posedge pclk);
        data = prdata;
        @(posedge pclk);
        psel    <= 1'b0;
        penable <= 1'b0;
    endtask

    logic [31:0] read_val;

    initial begin
        $display("==================================================");
        $display("    TESTBENCH: APB4 Protocol Bridge (Unit)       ");
        $display("==================================================");

        // Reset
        presetn = 0;
        paddr   = 0;
        psel    = 0;
        penable = 0;
        pwrite  = 0;
        pwdata  = 0;
        pstrb   = 0;
        pprot   = 0;

        #40;
        presetn = 1;
        #20;

        // Test 1: Single Register Write & Read
        $display("[TEST 1] APB single write and read at word 0x01 (byte 0x04)...");
        apb_write(11'h004, 32'h0000_0002); // Mode = AltHold (2)
        apb_read(11'h004, read_val);

        if (read_val !== 32'h0000_0002) begin
            $display("[FAIL] Readback mismatch! Expected: 0x00000002, Got: 0x%08h", read_val);
            error_count++;
        end else begin
            $display("[PASS] APB readback verified: 0x%08h", read_val);
        end

        // Test 2: Multi-register sweep
        $display("[TEST 2] APB multiple register sweep (words 0x10..0x15)...");
        begin
            logic [ADDR_W-1:0] baddr;
            logic [31:0] pattern;
            for (int w = 16; w <= 21; w++) begin
                baddr   = ADDR_W'(w * 4);
                pattern = 32'hB000_0000 | (32'(w) << 12) | (32'(w) * 31);
                apb_write(baddr, pattern);
                apb_read(baddr, read_val);
                if (read_val !== pattern) begin
                    $display("[FAIL] Word 0x%02h mismatch! Expected: 0x%08h, Got: 0x%08h", w, pattern, read_val);
                    error_count++;
                end
            end
        end

        #40;
        if (error_count == 0) begin
            $display("==================================================");
            $display("    [PASS] ALL APB4 BRIDGE TESTS PASSED           ");
            $display("==================================================");
            $finish(0);
        end else begin
            $display("==================================================");
            $display("    [FAIL] %0d ERRORS DETECTED IN APB4 TB         ", error_count);
            $display("==================================================");
            $fatal(1);
        end
    end

endmodule : pid_apb4_bridge_tb

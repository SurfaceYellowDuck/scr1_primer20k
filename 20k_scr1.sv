module primer20k_scr1 (
    // CLK AND RST
    input  logic                    CLK_G_50,  // 50MHz crystal oscillator Input
    inout  logic                    RESET_N,   // External clock input
    input  logic                    CLK_SMA, // 
    
    // DDR3 SDRAM
    // output logic                    DDR3_CK_N
    // output logic                    DDR3_CK_P
    // output logic                    DDR3_CK_EN
    // output logic                    DDR3_RST_N
    // output logic                    DDR3_CS_N
    // output logic                    DDR3_WE_N
    // output logic                    DDR3_RAS_N
    // output logic                    DDR3_CAS_N
    // output logic [2:0]              DDR3_BA 
    // output logic [13:0]             DDR3_A 
    // output logic                    DDR3_UDM
    // output logic                    DDR3_LDM
    // inout logic [15:0]              DDR3_DQ 
    // inout logic                     DDR3_UDQS_P
    // inout logic                     DDR3_LDQS_P
    // inout logic                     DDR3_UDQS_N
    // inout logic                     DDR3_LDQS_N
    // output logic                    DDR3_ODT
    
    // jtag


    // UART
	output logic                       uart_tx,
    input logic                      uart_rx
);

logic                   pwrup_rst_n;
logic                   cpu_clk;
logic                   extn_rst_in_n;
logic                   extn_rst_n;
logic [1:0]             extn_rst_n_sync;
logic                   hard_rst_n;
logic [3:0]             hard_rst_n_count;
logic                   soc_rst_n;
logic                   cpu_rst_n;
`ifdef SCR1_DBG_EN
logic                   sys_rst_n;
`endif // SCR1_DBG_EN


// --- SCR1 ---------------------------------------------
// AHB
logic [3:0]                         ahb_imem_hprot;
logic [2:0]                         ahb_imem_hburst;
logic [2:0]                         ahb_imem_hsize;
logic [1:0]                         ahb_imem_htrans;
//logic                               ahb_imem_hmastlock;
logic [SCR1_AHB_WIDTH-1:0]          ahb_imem_haddr;
logic                               ahb_imem_hready;
logic [SCR1_AHB_WIDTH-1:0]          ahb_imem_hrdata;
logic                               ahb_imem_hresp;
//
logic [3:0]                         ahb_dmem_hprot;
logic [2:0]                         ahb_dmem_hburst;
logic [2:0]                         ahb_dmem_hsize;
logic [1:0]                         ahb_dmem_htrans;
//logic                               ahb_dmem_hmastlock;
logic [SCR1_AHB_WIDTH-1:0]          ahb_dmem_haddr;
logic                               ahb_dmem_hwrite;
logic [SCR1_AHB_WIDTH-1:0]          ahb_dmem_hwdata;
logic                               ahb_dmem_hready;
logic [SCR1_AHB_WIDTH-1:0]          ahb_dmem_hrdata;
logic                               ahb_dmem_hresp;

`ifdef SCR1_IPIC_EN
logic [SCR1_IRQ_LINES_NUM-1:0]            scr1_irq;
`else
logic                                     scr1_irq;
`endif // SCR1_IPIC_EN

// // --- UART ---------------------------------------------
// logic                               uart_rxd;   // -> UART
// logic                               uart_txd;   // <- UART
logic                               uart_rts_n; // <- UART
logic                               uart_dtr_n; // <- UART
// logic                               uart_irq;

// --- Heartbeat ----------------------------------------
logic [31:0]                        rtc_counter;
logic                               tick_2Hz;
logic                               heartbeat;

//=======================================================
//  Structural coding
//=======================================================

//=======================================================
//  Resets
//=======================================================
assign extn_rst_in_n    = RESET_N


always_ff @(posedge cpu_clk, negedge pwrup_rst_n)
begin
    if (~pwrup_rst_n) begin
        extn_rst_n_sync     <= '0;
    end else begin
        extn_rst_n_sync[0]  <= extn_rst_in_n;
        extn_rst_n_sync[1]  <= extn_rst_n_sync[0];
    end
end
assign extn_rst_n = extn_rst_n_sync[1];

always_ff @(posedge cpu_clk, negedge pwrup_rst_n)
begin
    if (~pwrup_rst_n) begin
        hard_rst_n          <= 1'b0;
        hard_rst_n_count    <= '0;
    end else begin
        if (hard_rst_n) begin
            // hard_rst_n == 1 - de-asserted
            hard_rst_n          <= extn_rst_n;
            hard_rst_n_count    <= '0;
        end else begin
            // hard_rst_n == 0 - asserted
            if (extn_rst_n) begin
                if (hard_rst_n_count == '1) begin
                    // If extn_rst_n = 1 at least 16 clocks,
                    // de-assert hard_rst_n
                    hard_rst_n          <= 1'b1;
                end else begin
                    hard_rst_n_count    <= hard_rst_n_count + 1;
                end
            end else begin
                // If extn_rst_n is asserted within 16-cycles window -> start
                // counting from the beginning
                hard_rst_n_count    <= '0;
            end
        end
    end
end

uart_top
i_uart(
    .clk            (cpu_clk),
    .rst_n          (uart_rts_n),
    .uart_rx        (uart_rx),
    .uart_tx        (uart_tx)
);


`ifdef SCR1_DBG_EN
assign soc_rst_n = sys_rst_n;
`else
assign soc_rst_n = hard_rst_n;
`endif // SCR1_DBG_EN

//=======================================================
//  Heartbeat
//=======================================================
always_ff @(posedge cpu_clk, negedge hard_rst_n)
begin
    if (~hard_rst_n) begin
        rtc_counter     <= '0;
        tick_2Hz        <= 1'b0;
    end
    else begin
        if (rtc_counter == '0) begin
            rtc_counter <= (FPGA_PRIMER_20K_CORE_CLK_FREQ/2);
            tick_2Hz    <= 1'b1;
        end
        else begin
            rtc_counter <= rtc_counter - 1'b1;
            tick_2Hz    <= 1'b0;
        end
    end
end

always_ff @(posedge cpu_clk, negedge hard_rst_n)
begin
    if (~hard_rst_n) begin
        heartbeat       <= 1'b0;
    end
    else begin
        if (tick_2Hz) begin
            heartbeat   <= ~heartbeat;
        end
    end
end


//=======================================================
//  SCR1 Core's Processor Cluster
//=======================================================
scr1_top_axi
i_scr_top (
    // Common
    .pwrup_rst_n                        (pwrup_rst_n            ),
    .rst_n                              (hard_rst_n             ),
    .cpu_rst_n                          (cpu_rst_n              ),
    .test_mode                          (1'b0                   ),
    .test_rst_n                         (1'b1                   ),
    .clk                                (cpu_clk                ),
    .rtc_clk                            (1'b0                   ),
`ifdef SCR1_DBG_EN
    .sys_rst_n_o                        (sys_rst_n              ),
    .sys_rdc_qlfy_o                     (                       ),
`endif // SCR1_DBG_EN

    // Fuses
    .fuse_mhartid                       ('0                     ),
`ifdef SCR1_DBG_EN
    .fuse_idcode                        (`SCR1_TAP_IDCODE       ),
`endif // SCR1_DBG_EN

`ifdef SCR1_IPIC_EN
    .irq_lines                          (scr1_irq               ),
`else // SCR1_IPIC_EN
    .ext_irq                            (scr1_irq               ),
`endif // SCR1_IPIC_EN
    .soft_irq                           ('0                     ),
    // Debug Interface
`ifdef SCR1_DBG_EN
    .trst_n                             (scr1_jtag_trst_n       ),
    .tck                                (scr1_jtag_tck          ),
    .tms                                (scr1_jtag_tms          ),
    .tdi                                (scr1_jtag_tdi          ),
    .tdo                                (scr1_jtag_tdo_int      ),
    .tdo_en                             (scr1_jtag_tdo_en       ),
`endif // SCR1_DBG_EN

 // Instruction Memory Interface
        .imem_hprot                 (ahb_imem_hprot         ),
        .imem_hburst                (ahb_imem_hburst        ),
        .imem_hsize                 (ahb_imem_hsize         ),
        .imem_htrans                (ahb_imem_htrans        ),
        .imem_hmastlock             (                       ),
        .imem_haddr                 (ahb_imem_haddr         ),
        .imem_hready                (ahb_imem_hready        ),
        .imem_hrdata                (ahb_imem_hrdata        ),
        .imem_hresp                 (ahb_imem_hresp         ),
        // Data Memory Interface
        .dmem_hprot                 (ahb_dmem_hprot         ),
        .dmem_hburst                (ahb_dmem_hburst        ),
        .dmem_hsize                 (ahb_dmem_hsize         ),
        .dmem_htrans                (ahb_dmem_htrans        ),
        .dmem_hmastlock             (                       ),
        .dmem_haddr                 (ahb_dmem_haddr         ),
        .dmem_hwrite                (ahb_dmem_hwrite        ),
        .dmem_hwdata                (ahb_dmem_hwdata        ),
        .dmem_hready                (ahb_dmem_hready        ),
        .dmem_hrdata                (ahb_dmem_hrdata        ),
        .dmem_hresp                 (ahb_dmem_hresp         )
);

assign scr1_irq = {31'd0};

endmodule


`default_nettype none

module thinpad_top (
    input wire clk_50M,     // 50MHz 时钟输入
    input wire clk_11M0592, // 11.0592MHz 时钟输入（备用，可不用）

    input wire push_btn,  // BTN5 按钮开关，带消抖电路，按下时为 1
    input wire reset_btn, // BTN6 复位按钮，带消抖电路，按下时为 1

    input  wire [ 3:0] touch_btn,  // BTN1~BTN4，按钮开关，按下时为 1
    input  wire [31:0] dip_sw,     // 32 位拨码开关，拨到“ON”时为 1
    output wire [15:0] leds,       // 16 位 LED，输出时 1 点亮
    output wire [ 7:0] dpy0,       // 数码管低位信号，包括小数点，输出 1 点亮
    output wire [ 7:0] dpy1,       // 数码管高位信号，包括小数点，输出 1 点亮

    // CPLD 串口控制器信号
    output wire uart_rdn,        // 读串口信号，低有效
    output wire uart_wrn,        // 写串口信号，低有效
    input  wire uart_dataready,  // 串口数据准备好
    input  wire uart_tbre,       // 发送数据标志
    input  wire uart_tsre,       // 数据发送完毕标志

    // BaseRAM 信号
    inout wire [31:0] base_ram_data,  // BaseRAM 数据，低 8 位与 CPLD 串口控制器共享
    output wire [19:0] base_ram_addr,  // BaseRAM 地址
    output wire [3:0] base_ram_be_n,  // BaseRAM 字节使能，低有效。如果不使用字节使能，请保持为 0
    output wire base_ram_ce_n,  // BaseRAM 片选，低有效
    output wire base_ram_oe_n,  // BaseRAM 读使能，低有效
    output wire base_ram_we_n,  // BaseRAM 写使能，低有效

    // ExtRAM 信号
    inout wire [31:0] ext_ram_data,  // ExtRAM 数据
    output wire [19:0] ext_ram_addr,  // ExtRAM 地址
    output wire [3:0] ext_ram_be_n,  // ExtRAM 字节使能，低有效。如果不使用字节使能，请保持为 0
    output wire ext_ram_ce_n,  // ExtRAM 片选，低有效
    output wire ext_ram_oe_n,  // ExtRAM 读使能，低有效
    output wire ext_ram_we_n,  // ExtRAM 写使能，低有效

    // 直连串口信号
    output wire txd,  // 直连串口发送端
    input  wire rxd,  // 直连串口接收端

    // Flash 存储器信号，参考 JS28F640 芯片手册
    output wire [22:0] flash_a,  // Flash 地址，a0 仅在 8bit 模式有效，16bit 模式无意义
    inout wire [15:0] flash_d,  // Flash 数据
    output wire flash_rp_n,  // Flash 复位信号，低有效
    output wire flash_vpen,  // Flash 写保护信号，低电平时不能擦除、烧写
    output wire flash_ce_n,  // Flash 片选信号，低有效
    output wire flash_oe_n,  // Flash 读使能信号，低有效
    output wire flash_we_n,  // Flash 写使能信号，低有效
    output wire flash_byte_n, // Flash 8bit 模式选择，低有效。在使用 flash 的 16 位模式时请设为 1

    // USB 控制器信号，参考 SL811 芯片手册
    output wire sl811_a0,
    // inout  wire [7:0] sl811_d,     // USB 数据线与网络控制器的 dm9k_sd[7:0] 共享
    output wire sl811_wr_n,
    output wire sl811_rd_n,
    output wire sl811_cs_n,
    output wire sl811_rst_n,
    output wire sl811_dack_n,
    input  wire sl811_intrq,
    input  wire sl811_drq_n,

    // 网络控制器信号，参考 DM9000A 芯片手册
    output wire dm9k_cmd,
    inout wire [15:0] dm9k_sd,
    output wire dm9k_iow_n,
    output wire dm9k_ior_n,
    output wire dm9k_cs_n,
    output wire dm9k_pwrst_n,
    input wire dm9k_int,

    // 图像输出信号
    output wire [2:0] video_red,    // 红色像素，3 位
    output wire [2:0] video_green,  // 绿色像素，3 位
    output wire [1:0] video_blue,   // 蓝色像素，2 位
    output wire       video_hsync,  // 行同步（水平同步）信号
    output wire       video_vsync,  // 场同步（垂直同步）信号
    output wire       video_clk,    // 像素时钟输出
    output wire       video_de      // 行数据有效信号，用于区分消隐区
);

  logic sys_clk;
  logic sys_rst;

  assign sys_clk = clk_10M;
  assign sys_rst = reset_of_clk10M;

  // 本实验不使用 CPLD 串口，禁用防止总线冲突
  assign uart_rdn = 1'b1;
  assign uart_wrn = 1'b1;

  /* =========== Lab5 Master begin =========== */
  // Lab5 Master => Wishbone MUX (Slave)
  logic        wbm_cyc_o;
  logic        wbm_stb_o;
  logic        wbm_ack_i;
  logic [31:0] wbm_adr_o;
  logic [31:0] wbm_dat_o;
  logic [31:0] wbm_dat_i;
  logic [ 3:0] wbm_sel_o;
  logic        wbm_we_o;

  // user-defined
  logic        top_cyc_o;
  logic        top_stb_o;
  logic        top_we_o;
	logic [31:0] top_adr_o;
	logic [ 3:0] top_sel_o;
	logic        top_ack_i;
	logic [31:0] top_dat_o;
	logic [31:0] top_dat_i;

  // Lab 6 begin ----------------------------------------------------------------------------------
  typedef enum logic [3:0] {
    STATE_IF,
    STATE_ID,
    STATE_EXE,
    STATE_MEM,
    STATE_WB
  } state_t;

  state_t state;

  reg [31:0] pc_reg = 32'h8000_0000;
  reg [31:0] pc_now_reg;
  reg [31:0] inst_reg;

  // alu
  reg [31:0] alu_operand1_o;
  reg [31:0] alu_operand2_o;
  reg [ 3:0] alu_op_o;
  reg [31:0] alu_result_i;

  // register file
  logic [ 4:0] rf_raddr_a_o;
  logic [31:0] rf_rdata_a_i;
  logic [ 4:0] rf_raddr_b_o;
  logic [31:0] rf_rdata_b_i;
  logic [ 4:0] rf_waddr_o;
  logic [31:0] rf_wdata_o;
  logic rf_we_o;

  logic [31:0] imm;

  // ID stage
  reg [31:0] operand1_reg;
  reg [31:0] operand2_reg;

  // EXE stage
  reg [31:0] rf_writeback_reg;

  //       [14:12] [6:0]
  //  LUI:         0110111
  //  BEQ:     000 1100011
  //   LB:     000 0000011
  //   SB:     000 0100011
  //   SW:     010 0100011
  // ADDI:     000 0010011 √
  // ANDI:     111 0010011 √        
  //  ADD:     000 0110011 √

  always_comb begin
    case (state)

      STATE_IF: begin
        top_adr_o = pc_reg;
        top_cyc_o = 1'b1;
        alu_operand1_o = pc_reg;
        alu_operand2_o = 32'h0000_0004;
        alu_op_o = 4'b0001; // PC <- PC + 4
      end

      STATE_ID: begin
        if (inst_reg[6:0] == 7'b0010011) begin // ADDI & ANDI
          rf_raddr_a_o = inst_reg[19:15]; // rs1
          imm = inst_reg[31:20];
        end else if (inst_reg[6:0] == 7'b0110011) begin // ADD
          rf_raddr_a_o = inst_reg[19:15]; // rs1
          rf_raddr_b_o = inst_reg[24:20]; // rs2
        end
      end

      STATE_EXE: begin
        alu_operand1_o = operand1_reg;
        alu_operand2_o = operand2_reg;
        if ((inst_reg[6:0] == 7'b0010011 && inst_reg[14:12] == 3'b000) || inst_reg[6:0] == 7'b0110011) begin // ADDI & ADD
          alu_op_o = 4'b0001; // ALU_ADD
        end else if (inst_reg[6:0] == 7'b0010011 && inst_reg[14:12] == 3'b111) begin // ANDI
          alu_op_o = 4'b0011; // ALU_AND
        end
      end

      STATE_WB: begin
        if (inst_reg[6:0] == 7'b0010011 || inst_reg[6:0] == 7'b0110011) begin // ADDI & ANDI & ADD
          rf_we_o = 1'b1;
          rf_wdata_o = rf_writeback_reg;
          rf_waddr_o = inst_reg[11:7]; // rd
        end
      end

      default: begin
      end
    endcase
  end

  always_ff @ (posedge sys_clk) begin
    case (state)

      STATE_IF: begin
        inst_reg <= top_dat_i;
        top_cyc_o <= 1'b0;
        pc_now_reg <= pc_reg; // pc_now_reg: this instr, pc_reg: next instr
        if (wb_ack_i) begin // wishbone ack: PC + 4 get
          pc_reg <= alu_result_i; // hold `addr` when wishbone request
          state <= STATE_ID;
        end
      end

      STATE_ID: begin
        if (inst_reg[6:0] == 7'b0010011) begin // ADDI $ ANDI
          operand1_reg <= rf_rdata_a_i; // rs1
          operand2_reg <= imm; // rd
        end else if (inst_reg[6:0] == 7'b0110011) begin // ADD
          operand1_reg <= rf_rdata_a_i; // rs1
          operand2_reg <= rf_rdata_b_i; // rs2
        end
        state <= STATE_EXE;
      end

      STATE_EXE: begin
        if (inst_reg[6:0] == 7'b0010011 || inst_reg[6:0] == 7'b0110011) begin // ADDI & ANDI & ADD
          rf_writeback_reg <= alu_result_i;
          state <= STATE_WB;
        end
      end

      STATE_WB: begin
        state <= STATE_IF;
      end

      default: begin
      end
    endcase
  end

  alu_32 u_alu_32 (
    .a  (alu_operand1_o),
    .b  (alu_operand2_o),
    .op (alu_op_o),
    .y  (alu_result_i)
  );

  register_file_32 u_register_file_32 (
      .clk     (sys_clk),
      .reset   (sys_rst),
      .waddr   (rf_waddr_o),
      .wdata   (rf_wdata_o),
      .we      (rf_we_o),
      .raddr_a (rf_raddr_a_o),
      .rdata_a (rf_rdata_a_i),
      .raddr_b (rf_raddr_b_o),
      .rdata_b (rf_rdata_b_i)
  );

  // Lab 6 end ------------------------------------------------------------------------------------

  // Lab 5 modules
  lab5_master #(
      .ADDR_WIDTH(32),
      .DATA_WIDTH(32)
  ) u_lab5_master (
      .clk_i(sys_clk),
      .rst_i(sys_rst),

      // TODO: 添加需要的控制信号，例如按键开关？
      .top_adr(top_adr_o),
      .top_cyc(top_cyc_o),
      .top_stb(top_stb_o),
      .top_we(top_we_o),
			.top_sel(top_sel_o),
			.top_ack(top_ack_i),
			.top_dat_i(top_dat_o),
    	.top_dat_o(top_dat_i),

      // wishbone master
      .wb_cyc_o(wbm_cyc_o),
      .wb_stb_o(wbm_stb_o),
      .wb_ack_i(wbm_ack_i),
      .wb_adr_o(wbm_adr_o),
      .wb_dat_o(wbm_dat_o),
      .wb_dat_i(wbm_dat_i),
      .wb_sel_o(wbm_sel_o),
      .wb_we_o (wbm_we_o)
  );

  /* =========== Lab5 Master end =========== */

  /* =========== Lab5 MUX begin =========== */
  // Wishbone MUX (Masters) => bus slaves
  logic wbs0_cyc_o;
  logic wbs0_stb_o;
  logic wbs0_ack_i;
  logic [31:0] wbs0_adr_o;
  logic [31:0] wbs0_dat_o;
  logic [31:0] wbs0_dat_i;
  logic [3:0] wbs0_sel_o;
  logic wbs0_we_o;

  logic wbs1_cyc_o;
  logic wbs1_stb_o;
  logic wbs1_ack_i;
  logic [31:0] wbs1_adr_o;
  logic [31:0] wbs1_dat_o;
  logic [31:0] wbs1_dat_i;
  logic [3:0] wbs1_sel_o;
  logic wbs1_we_o;

  logic wbs2_cyc_o;
  logic wbs2_stb_o;
  logic wbs2_ack_i;
  logic [31:0] wbs2_adr_o;
  logic [31:0] wbs2_dat_o;
  logic [31:0] wbs2_dat_i;
  logic [3:0] wbs2_sel_o;
  logic wbs2_we_o;

  wb_mux_3 wb_mux (
      .clk(sys_clk),
      .rst(sys_rst),

      // Master interface (to Lab5 master)
      .wbm_adr_i(wbm_adr_o),
      .wbm_dat_i(wbm_dat_o),
      .wbm_dat_o(wbm_dat_i),
      .wbm_we_i (wbm_we_o),
      .wbm_sel_i(wbm_sel_o),
      .wbm_stb_i(wbm_stb_o),
      .wbm_ack_o(wbm_ack_i),
      .wbm_err_o(),
      .wbm_rty_o(),
      .wbm_cyc_i(wbm_cyc_o),

      // Slave interface 0 (to BaseRAM controller)
      // Address range: 0x8000_0000 ~ 0x803F_FFFF
      .wbs0_addr    (32'h8000_0000),
      .wbs0_addr_msk(32'hFFC0_0000),

      .wbs0_adr_o(wbs0_adr_o),
      .wbs0_dat_i(wbs0_dat_i),
      .wbs0_dat_o(wbs0_dat_o),
      .wbs0_we_o (wbs0_we_o),
      .wbs0_sel_o(wbs0_sel_o),
      .wbs0_stb_o(wbs0_stb_o),
      .wbs0_ack_i(wbs0_ack_i),
      .wbs0_err_i('0),
      .wbs0_rty_i('0),
      .wbs0_cyc_o(wbs0_cyc_o),

      // Slave interface 1 (to ExtRAM controller)
      // Address range: 0x8040_0000 ~ 0x807F_FFFF
      .wbs1_addr    (32'h8040_0000),
      .wbs1_addr_msk(32'hFFC0_0000),

      .wbs1_adr_o(wbs1_adr_o),
      .wbs1_dat_i(wbs1_dat_i),
      .wbs1_dat_o(wbs1_dat_o),
      .wbs1_we_o (wbs1_we_o),
      .wbs1_sel_o(wbs1_sel_o),
      .wbs1_stb_o(wbs1_stb_o),
      .wbs1_ack_i(wbs1_ack_i),
      .wbs1_err_i('0),
      .wbs1_rty_i('0),
      .wbs1_cyc_o(wbs1_cyc_o),

      // Slave interface 2 (to UART controller)
      // Address range: 0x1000_0000 ~ 0x1000_FFFF
      .wbs2_addr    (32'h1000_0000),
      .wbs2_addr_msk(32'hFFFF_0000),

      .wbs2_adr_o(wbs2_adr_o),
      .wbs2_dat_i(wbs2_dat_i),
      .wbs2_dat_o(wbs2_dat_o),
      .wbs2_we_o (wbs2_we_o),
      .wbs2_sel_o(wbs2_sel_o),
      .wbs2_stb_o(wbs2_stb_o),
      .wbs2_ack_i(wbs2_ack_i),
      .wbs2_err_i('0),
      .wbs2_rty_i('0),
      .wbs2_cyc_o(wbs2_cyc_o)
  );

  /* =========== Lab5 MUX end =========== */

  /* =========== Lab5 Slaves begin =========== */
  sram_controller #(
      .SRAM_ADDR_WIDTH(20),
      .SRAM_DATA_WIDTH(32)
  ) sram_controller_base (
      .clk_i(sys_clk),
      .rst_i(sys_rst),

      // Wishbone slave (to MUX)
      .wb_cyc_i(wbs0_cyc_o),
      .wb_stb_i(wbs0_stb_o),
      .wb_ack_o(wbs0_ack_i),
      .wb_adr_i(wbs0_adr_o),
      .wb_dat_i(wbs0_dat_o),
      .wb_dat_o(wbs0_dat_i),
      .wb_sel_i(wbs0_sel_o),
      .wb_we_i (wbs0_we_o),

      // To SRAM chip
      .sram_addr(base_ram_addr),
      .sram_data(base_ram_data),
      .sram_ce_n(base_ram_ce_n),
      .sram_oe_n(base_ram_oe_n),
      .sram_we_n(base_ram_we_n),
      .sram_be_n(base_ram_be_n)
  );

  sram_controller #(
      .SRAM_ADDR_WIDTH(20),
      .SRAM_DATA_WIDTH(32)
  ) sram_controller_ext (
      .clk_i(sys_clk),
      .rst_i(sys_rst),

      // Wishbone slave (to MUX)
      .wb_cyc_i(wbs1_cyc_o),
      .wb_stb_i(wbs1_stb_o),
      .wb_ack_o(wbs1_ack_i),
      .wb_adr_i(wbs1_adr_o),
      .wb_dat_i(wbs1_dat_o),
      .wb_dat_o(wbs1_dat_i),
      .wb_sel_i(wbs1_sel_o),
      .wb_we_i (wbs1_we_o),

      // To SRAM chip
      .sram_addr(ext_ram_addr),
      .sram_data(ext_ram_data),
      .sram_ce_n(ext_ram_ce_n),
      .sram_oe_n(ext_ram_oe_n),
      .sram_we_n(ext_ram_we_n),
      .sram_be_n(ext_ram_be_n)
  );

  // 串口控制器模块
  // NOTE: 如果修改系统时钟频率，也需要修改此处的时钟频率参数
  uart_controller #(
      .CLK_FREQ(10_000_000),
      .BAUD    (115200)
  ) uart_controller (
      .clk_i(sys_clk),
      .rst_i(sys_rst),

      .wb_cyc_i(wbs2_cyc_o),
      .wb_stb_i(wbs2_stb_o),
      .wb_ack_o(wbs2_ack_i),
      .wb_adr_i(wbs2_adr_o),
      .wb_dat_i(wbs2_dat_o),
      .wb_dat_o(wbs2_dat_i),
      .wb_sel_i(wbs2_sel_o),
      .wb_we_i (wbs2_we_o),

      // to UART pins
      .uart_txd_o(txd),
      .uart_rxd_i(rxd)
  );

  /* =========== Lab5 Slaves end =========== */


endmodule

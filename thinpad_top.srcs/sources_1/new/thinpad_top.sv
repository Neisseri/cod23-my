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

  /* =========== Demo code begin =========== */

  // PLL 分频示例
  logic locked, clk_10M, clk_20M;
  pll_example clock_gen (
      // Clock in ports
      .clk_in1(clk_50M),  // 外部时钟输入
      // Clock out ports
      .clk_out1(clk_10M),  // 时钟输出 1，频率在 IP 配置界面中设置
      .clk_out2(clk_20M),  // 时钟输出 2，频率在 IP 配置界面中设置
      // Status and control signals
      .reset(reset_btn),  // PLL 复位输入
      .locked(locked)  // PLL 锁定指示输出，"1"表示时钟稳定，
                       // 后级电路复位信号应当由它生成（见下）
  );

  logic reset_of_clk10M;
  // 异步复位，同步释放，将 locked 信号转为后级电路的复位 reset_of_clk10M
  always_ff @(posedge clk_10M or negedge locked) begin
    if (~locked) reset_of_clk10M <= 1'b1;
    else reset_of_clk10M <= 1'b0;
  end

  always_ff @(posedge clk_10M or posedge reset_of_clk10M) begin
    if (reset_of_clk10M) begin
      // Your Code
    end else begin
      // Your Code
    end
  end

  // 数码管连接关系示意图，dpy1 同理
  // p=dpy0[0] // ---a---
  // c=dpy0[1] // |     |
  // d=dpy0[2] // f     b
  // e=dpy0[3] // |     |
  // b=dpy0[4] // ---g---
  // a=dpy0[5] // |     |
  // f=dpy0[6] // e     c
  // g=dpy0[7] // |     |
  //           // ---d---  p

  // 7 段数码管译码器演示，将 number 用 16 进制显示在数码管上面
  logic [7:0] number;
  SEG7_LUT segL (
      .oSEG1(dpy0),
      .iDIG (number[3:0])
  );  // dpy0 是低位数码管
  SEG7_LUT segH (
      .oSEG1(dpy1),
      .iDIG (number[7:4])
  );  // dpy1 是高位数码管

  logic [15:0] led_bits;
  assign leds = led_bits;

  always_ff @(posedge push_btn or posedge reset_btn) begin
    if (reset_btn) begin  // 复位按下，设置 LED 为初始值
      led_bits <= 16'h1;
    end else begin  // 每次按下按钮开关，LED 循环左移
      led_bits <= {led_bits[14:0], led_bits[15]};
    end
  end

  // 直连串口接收发送演示，从直连串口收到的数据再发送出去
  logic [7:0] ext_uart_rx;
  logic [7:0] ext_uart_buffer, ext_uart_tx;
  logic ext_uart_ready, ext_uart_clear, ext_uart_busy;
  logic ext_uart_start, ext_uart_avai;

  assign number = ext_uart_buffer;

  // 接收模块，9600 无检验位
  async_receiver #(
      .ClkFrequency(50000000),
      .Baud(9600)
  ) ext_uart_r (
      .clk           (clk_50M),         // 外部时钟信号
      .RxD           (rxd),             // 外部串行信号输入
      .RxD_data_ready(ext_uart_ready),  // 数据接收到标志
      .RxD_clear     (ext_uart_clear),  // 清除接收标志
      .RxD_data      (ext_uart_rx)      // 接收到的一字节数据
  );

  assign ext_uart_clear = ext_uart_ready; // 收到数据的同时，清除标志，因为数据已取到 ext_uart_buffer 中
  always_ff @(posedge clk_50M) begin  // 接收到缓冲区 ext_uart_buffer
    if (ext_uart_ready) begin
      ext_uart_buffer <= ext_uart_rx;
      ext_uart_avai   <= 1;
    end else if (!ext_uart_busy && ext_uart_avai) begin
      ext_uart_avai <= 0;
    end
  end
  always_ff @(posedge clk_50M) begin  // 将缓冲区 ext_uart_buffer 发送出去
    if (!ext_uart_busy && ext_uart_avai) begin
      ext_uart_tx <= ext_uart_buffer;
      ext_uart_start <= 1;
    end else begin
      ext_uart_start <= 0;
    end
  end

  // 图像输出演示，分辨率 800x600@75Hz，像素时钟为 50MHz
  logic [11:0] hdata;
  assign video_red   = hdata < 266 ? 3'b111 : 0;  // 红色竖条
  assign video_green = hdata < 532 && hdata >= 266 ? 3'b111 : 0;  // 绿色竖条
  assign video_blue  = hdata >= 532 ? 2'b11 : 0;  // 蓝色竖条
  assign video_clk   = clk_50M;
  vga #(12, 800, 856, 976, 1040, 600, 637, 643, 666, 1, 1) vga800x600at75 (
      .clk        (clk_50M),
      .hdata      (hdata),        // 横坐标
      .vdata      (),             // 纵坐标
      .hsync      (video_hsync),
      .vsync      (video_vsync),
      .data_enable(video_de)
  );
  /* =========== Demo code end =========== */

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
    STATE_WB,
    STATE_END
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

  // MEM stage
  reg [31:0] sram_addr_reg;

  //  load: https://blog.csdn.net/sucaiwa/article/details/129328907
  // store: https://blog.csdn.net/sucaiwa/article/details/129328939

  //       [14:12] [6:0]
  //  LUI:         0110111 √

  //  BEQ:     000 1100011

  //   LB:     000 0000011 √

  //   SB:     000 0100011 √
  //   SW:     010 0100011 √

  // ADDI:     000 0010011 √
  // ANDI:     111 0010011 √

  //  ADD:     000 0110011 √

  always_comb begin
    case (state)

      STATE_IF: begin
        top_cyc_o = 1'b1;
        top_stb_o = 1'b1;
        top_adr_o = pc_reg;
        top_dat_o = 32'h0000_0000;
        top_sel_o = 4'b1111;
        top_we_o = 1'b0; // read
        
        imm = 32'h0000_0000;
        
        rf_raddr_a_o = 5'b00000;
        rf_raddr_b_o = 5'b00000;
        rf_waddr_o = 5'b00000;
        rf_wdata_o = 32'h0000_0000;
        rf_we_o = 1'b0;

        alu_operand1_o = pc_reg;
        alu_operand2_o = 32'h0000_0004;
        alu_op_o = 4'b0001; // PC <- PC + 4
      end

      STATE_ID: begin
        // to avoid latch
        top_cyc_o = 1'b0;
        top_stb_o = 1'b0;
        top_adr_o = 32'h0000_0000;
        top_dat_o = 32'h0000_0000;
        top_sel_o = 4'b0000;
        top_we_o = 1'b0;
        
        alu_operand1_o = 32'h0000_0000;
        alu_operand2_o = 32'h0000_0000;
        alu_op_o = 4'b0000;

        if (inst_reg[6:0] == 7'b0010011) begin // ADDI & ANDI
          imm = $signed(inst_reg[31:20]);

          rf_raddr_a_o = inst_reg[19:15]; // rs1
          rf_raddr_b_o = 5'b00000;
          rf_waddr_o = 5'b00000;
          rf_wdata_o = 32'h0000_0000;
          rf_we_o = 1'b0;
        end else if (inst_reg[6:0] == 7'b0110011) begin // ADD
          imm = 32'h0000_0000;

          rf_raddr_a_o = inst_reg[19:15]; // rs1
          rf_raddr_b_o = inst_reg[24:20]; // rs2
          rf_waddr_o = 5'b00000;
          rf_wdata_o = 32'h0000_0000;
          rf_we_o = 1'b0;
        end else if (inst_reg[6:0] == 7'b0100011) begin // SW & SB
          imm = $signed({inst_reg[31:25], inst_reg[11:7]});
          //             [4:0]   [11:5]

          rf_raddr_a_o = inst_reg[19:15]; // rs1
          rf_raddr_b_o = inst_reg[24:20]; // rs2
          rf_waddr_o = 5'b00000;
          rf_wdata_o = 32'h0000_0000;
          rf_we_o = 1'b0;
        end else if (inst_reg[6:0] == 7'b0000011) begin // LB
          imm = $signed(inst_reg[31:20]);

          rf_raddr_a_o = inst_reg[19:15]; // rs1
          rf_raddr_b_o = 5'b00000;
          rf_waddr_o = 5'b00000;
          rf_wdata_o = 32'h0000_0000;
          rf_we_o = 1'b0;
        end else if (inst_reg[6:0] == 7'b0110111) begin // LUI
          imm = {inst_reg[31:12], 12'h000};
          
          rf_raddr_a_o = 5'b00000;
          rf_raddr_b_o = 5'b00000;
          rf_waddr_o = 5'b00000;
          rf_wdata_o = 32'h0000_0000;
          rf_we_o = 1'b0;
        end else if (inst_reg[6:0] == 7'b1100011) begin // BEQ
          imm = $signed({inst_reg[31], inst_reg[7], inst_reg[30:25], inst_reg[11:8], 1'b0});
          
          rf_raddr_a_o = inst_reg[19:15]; // rs1
          rf_raddr_b_o = inst_reg[24:20]; // rs2
          rf_waddr_o = 5'b00000;
          rf_wdata_o = 32'h0000_0000;
          rf_we_o = 1'b0;
        end else begin
          imm = 32'h0000_0000;

          rf_raddr_a_o = 5'b00000;
          rf_raddr_b_o = 5'b00000;
          rf_waddr_o = 5'b00000;
          rf_wdata_o = 32'h0000_0000;
          rf_we_o = 1'b0;
        end
      end

      STATE_EXE: begin
        top_cyc_o = 1'b0;
        top_stb_o = 1'b0;
        top_adr_o = 32'h0000_0000;
        top_dat_o = 32'h0000_0000;
        top_sel_o = 4'b0000;
        top_we_o = 1'b0;
        
        imm = 32'h0000_0000;

        rf_raddr_a_o = 5'b00000;
        rf_raddr_b_o = 5'b00000;
        rf_waddr_o = 5'b00000;
        rf_wdata_o = 32'h0000_0000;
        rf_we_o = 1'b0;
        
        alu_operand1_o = operand1_reg;
        alu_operand2_o = operand2_reg;

        if ((inst_reg[6:0] == 7'b0010011 && inst_reg[14:12] == 3'b000) || inst_reg[6:0] == 7'b0110011) begin // ADDI & ADD
          alu_op_o = 4'b0001; // ALU_ADD
        end else if (inst_reg[6:0] == 7'b0010011 && inst_reg[14:12] == 3'b111) begin // ANDI
          alu_op_o = 4'b0011; // ALU_AND
        end else if (inst_reg[6:0] == 7'b0100011 || inst_reg[6:0] == 7'b0000011) begin // SW & SB & LB
          alu_op_o = 4'b0001; // ALU_ADD
        end else begin
          alu_op_o = 4'b0000;
        end
      end

      STATE_MEM: begin
        imm = 32'h0000_0000;

        rf_raddr_a_o = 5'b00000;
        rf_waddr_o = 5'b00000;
        rf_wdata_o = 32'h0000_0000;
        rf_we_o = 1'b0;

        alu_operand1_o = 32'h0000_0000;
        alu_operand2_o = 32'h0000_0000;
        alu_op_o = 4'b0000;

        if (inst_reg[6:0] == 7'b0100011 && inst_reg[14:12] == 3'b010) begin // SW
          top_cyc_o = 1'b1;
          top_stb_o = 1'b1;
          top_adr_o = sram_addr_reg; // BaseRAM address: [rs1] + imm
          // note that in RAM, an address represents 32 bits
          top_dat_o = rf_rdata_b_i; // rs2
          top_sel_o = 4'b1111;
          top_we_o = 1'b1; // write

          rf_raddr_b_o = inst_reg[24:20]; // rs2
        end else if (inst_reg[6:0] == 7'b0100011 && inst_reg[14:12] == 3'b000) begin // SB
          top_cyc_o = 1'b1;
          top_stb_o = 1'b1;
          top_adr_o = sram_addr_reg; // [rs1] + imm
          top_dat_o = rf_rdata_b_i[7:0]; // rs2
          top_sel_o = 4'b0001;
          top_we_o = 1'b1; // write

          rf_raddr_b_o = inst_reg[24:20]; // rs2
        end else if (inst_reg[6:0] == 7'b0000011) begin // LB
          top_cyc_o = 1'b1;
          top_stb_o = 1'b1;
          top_adr_o = sram_addr_reg; // [rs1] + imm
          top_dat_o = 32'h0000_0000;
          top_sel_o = 4'b0001;
          top_we_o = 1'b0; // read

          rf_raddr_b_o = 5'b00000;
        end else begin
          top_cyc_o = 1'b0;
          top_stb_o = 1'b0;
          top_adr_o = 32'h0000_0000;
          top_dat_o = 32'h0000_0000;
          top_sel_o = 4'b0000;
          top_we_o = 1'b0;

          rf_raddr_b_o = 5'b00000;
        end
      end

      STATE_WB: begin
        top_cyc_o = 1'b0;
        top_stb_o = 1'b0;
        top_adr_o = 32'h0000_0000;
        top_dat_o = 32'h0000_0000;
        top_sel_o = 4'b0000;
        top_we_o = 1'b0;
        
        alu_operand1_o = 32'h0000_0000;
        alu_operand2_o = 32'h0000_0000;
        alu_op_o = 4'b0000;

        if (inst_reg[6:0] == 7'b0010011 || inst_reg[6:0] == 7'b0110011) begin // ADDI & ANDI & ADD
          imm = 32'h0000_0000;

          rf_raddr_a_o = 5'b00000;
          rf_raddr_b_o = 5'b00000;
          rf_waddr_o = inst_reg[11:7]; // rd
          rf_wdata_o = rf_writeback_reg;
          rf_we_o = 1'b1;
        end else if (inst_reg[6:0] == 7'b0000011) begin // LB
          imm = 32'h0000_0000;
          
          rf_raddr_a_o = 5'b00000;
          rf_raddr_b_o = 5'b00000;
          rf_waddr_o = inst_reg[11:7]; // rd
          rf_wdata_o = rf_writeback_reg;
          rf_we_o = 1'b1;
        end else if (inst_reg[6:0] == 7'b0110111) begin // LUI
          imm = {inst_reg[31:12], 12'h000};

          rf_raddr_a_o = 5'b00000;
          rf_raddr_b_o = 5'b00000;
          rf_waddr_o = inst_reg[11:7]; // rd
          rf_wdata_o = imm;
          rf_we_o = 1'b1;
        end else begin
          imm = 32'h0000_0000;

          rf_raddr_a_o = 5'b00000;
          rf_raddr_b_o = 5'b00000;
          rf_waddr_o = 5'b00000;
          rf_wdata_o = 32'h0000_0000;
          rf_we_o = 1'b0;
        end
      end

      STATE_END: begin
        top_cyc_o = 1'b0;
        top_stb_o = 1'b0;
        top_adr_o = 32'h0000_0000;
        top_dat_o = 32'h0000_0000;
        top_sel_o = 4'b0000;
        top_we_o = 1'b0;
        
        imm = 32'h0000_0000;

        rf_raddr_a_o = 5'b00000;
        rf_raddr_b_o = 5'b00000;
        rf_waddr_o = 5'b00000;
        rf_wdata_o = 32'h0000_0000;
        rf_we_o = 1'b0;
        
        alu_operand1_o = 32'h0000_0000;
        alu_operand2_o = 32'h0000_0000;
        alu_op_o = 4'b0000;
      end

      default: begin
        top_cyc_o = 1'b0;
        top_stb_o = 1'b0;
        top_adr_o = 32'h0000_0000;
        top_dat_o = 32'h0000_0000;
        top_sel_o = 4'b0000;
        top_we_o = 1'b0;
        
        imm = 32'h0000_0000;

        rf_raddr_a_o = 5'b00000;
        rf_raddr_b_o = 5'b00000;
        rf_waddr_o = 5'b00000;
        rf_wdata_o = 32'h0000_0000;
        rf_we_o = 1'b0;
        
        alu_operand1_o = 32'h0000_0000;
        alu_operand2_o = 32'h0000_0000;
        alu_op_o = 4'b0000;
      end
    endcase
  end

  always_ff @ (posedge sys_clk) begin
    
    if (reset_btn) begin
      state <= STATE_IF;
    end else begin
      case (state)

        STATE_IF: begin
          if (top_ack_i == 1'b1) begin // reading inst finished
            inst_reg <= top_dat_i;
            // top_cyc_o <= 1'b0;
            // top_stb_o <= 1'b0;

            pc_now_reg <= pc_reg; // pc_now_reg: this instr, pc_reg: next instr
            pc_reg <= alu_result_i; // hold `addr` when wishbone request
            state <= STATE_ID;
          end
        end

        STATE_ID: begin
          if (inst_reg[6:0] == 7'b0010011) begin // ADDI $ ANDI
            operand1_reg <= rf_rdata_a_i; // rs1
            operand2_reg <= imm; // imm
            state <= STATE_EXE;
          end else if (inst_reg[6:0] == 7'b0110011) begin // ADD
            operand1_reg <= rf_rdata_a_i; // rs1
            operand2_reg <= rf_rdata_b_i; // rs2
            state <= STATE_EXE;
          end else if (inst_reg[6:0] == 7'b0100011) begin // SW & SB
            operand1_reg <= rf_rdata_a_i; // rs1
            operand2_reg <= imm; // signed extension
            state <= STATE_EXE;
          end else if (inst_reg[6:0] == 7'b0000011) begin // LB
            operand1_reg <= rf_rdata_a_i; // rs1
            operand2_reg <= imm;
            state <= STATE_EXE;
          end else if (inst_reg[6:0] == 7'b0110111) begin // LUI
            state <= STATE_WB;
          end else if (inst_reg[6:0] == 7'b1100011) begin // BEQ
            if (rf_rdata_a_i == rf_rdata_b_i) begin
              pc_reg <= pc_now_reg + imm;
            end
            state <= STATE_IF;
          end
        end

        STATE_EXE: begin
          if (inst_reg[6:0] == 7'b0010011 || inst_reg[6:0] == 7'b0110011) begin // ADDI & ANDI & ADD
            rf_writeback_reg <= alu_result_i;
            state <= STATE_WB;
          end else if (inst_reg[6:0] == 7'b0100011 || inst_reg[6:0] == 7'b0000011) begin // SW & SB & LB
            sram_addr_reg <= alu_result_i;
            state <= STATE_MEM;
          end
        end

        STATE_MEM: begin
          if (top_ack_i == 1'b1) begin // write or read is finished
            if (inst_reg[6:0] == 7'b0100011) begin // SW & SB
              // top_cyc_o <= 1'b0;
              // top_stb_o <= 1'b0;
              state <= STATE_END;
            end else if (inst_reg[6:0] == 7'b0000011) begin // LB
              rf_writeback_reg <= $signed(top_dat_i[7:0]);
              state <= STATE_WB;
            end
          end
        end

        STATE_WB: begin
          state <= STATE_IF;
        end

        STATE_END: begin
          state <= STATE_IF;
        end

        default: begin
        end
      endcase

    end
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

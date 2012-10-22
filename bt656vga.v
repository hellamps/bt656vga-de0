module bt656vga
	(
		////////////////////	Clock Input	 	////////////////////	 
		CLOCK_50,						//	50 MHz
		CLOCK_50_2,						//	50 MHz
		////////////////////	Push Button		////////////////////
		BUTTON,							//	Pushbutton[2:0]
		////////////////////	DPDT Switch		////////////////////
		SW,								//	Toggle Switch[9:0]
		////////////////////	7-SEG Dispaly	////////////////////
		HEX0_D,							//	Seven Segment Digit 0
		HEX0_DP,						//	Seven Segment Digit DP 0
		HEX1_D,							//	Seven Segment Digit 1
		HEX1_DP,						//	Seven Segment Digit DP 1
		HEX2_D,							//	Seven Segment Digit 2
		HEX2_DP,						//	Seven Segment Digit DP 2
		HEX3_D,							//	Seven Segment Digit 3
		HEX3_DP,						//	Seven Segment Digit DP 3
		////////////////////////	LED		////////////////////////
		LEDG,							//	LED Green[9:0]
		////////////////////////	UART	////////////////////////
		UART_TXD,						//	UART Transmitter
		UART_RXD,						//	UART Receiver
		UART_CTS,						//	UART Clear To Send
		UART_RTS,						//	UART Request To Send
		/////////////////////	SDRAM Interface		////////////////
		DRAM_DQ,						//	SDRAM Data bus 16 Bits
		DRAM_ADDR,						//	SDRAM Address bus 13 Bits
		DRAM_LDQM,						//	SDRAM Low-byte Data Mask 
		DRAM_UDQM,						//	SDRAM High-byte Data Mask
		DRAM_WE_N,						//	SDRAM Write Enable
		DRAM_CAS_N,						//	SDRAM Column Address Strobe
		DRAM_RAS_N,						//	SDRAM Row Address Strobe
		DRAM_CS_N,						//	SDRAM Chip Select
		DRAM_BA_0,						//	SDRAM Bank Address 0
		DRAM_BA_1,						//	SDRAM Bank Address 1
		DRAM_CLK,						//	SDRAM Clock
		DRAM_CKE,						//	SDRAM Clock Enable
		////////////////////	Flash Interface		////////////////
		FL_DQ,							//	FLASH Data bus 15 Bits
		FL_DQ15_AM1,					//	FLASH Data bus Bit 15 or Address A-1
		FL_ADDR,						//	FLASH Address bus 22 Bits
		FL_WE_N,						//	FLASH Write Enable
		FL_RST_N,						//	FLASH Reset
		FL_OE_N,						//	FLASH Output Enable
		FL_CE_N,						//	FLASH Chip Enable
		FL_WP_N,						//	FLASH Hardware Write Protect
		FL_BYTE_N,						//	FLASH Selects 8/16-bit mode
		FL_RY,							//	FLASH Ready/Busy
		////////////////////	LCD Module 16X2		////////////////
		LCD_BLON,						//	LCD Back Light ON/OFF
		LCD_RW,							//	LCD Read/Write Select, 0 = Write, 1 = Read
		LCD_EN,							//	LCD Enable
		LCD_RS,							//	LCD Command/Data Select, 0 = Command, 1 = Data
		LCD_DATA,						//	LCD Data bus 8 bits
		////////////////////	SD_Card Interface	////////////////
		SD_DAT0,						//	SD Card Data 0
		SD_DAT3,						//	SD Card Data 3
		SD_CMD,							//	SD Card Command Signal
		SD_CLK,							//	SD Card Clock
		SD_WP_N,						//	SD Card Write Protect
		////////////////////	PS2		////////////////////////////
		PS2_KBDAT,						//	PS2 Keyboard Data
		PS2_KBCLK,						//	PS2 Keyboard Clock
		PS2_MSDAT,						//	PS2 Mouse Data
		PS2_MSCLK,						//	PS2 Mouse Clock
		////////////////////	VGA		////////////////////////////
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_R,   						//	VGA Red[3:0]
		VGA_G,	 						//	VGA Green[3:0]
		VGA_B,  						//	VGA Blue[3:0]
		////////////////////	GPIO	////////////////////////////
		GPIO0_CLKIN,						//	GPIO Connection 0 Clock In Bus
		GPIO0_CLKOUT,					//	GPIO Connection 0 Clock Out Bus
		GPIO0_D,						//	GPIO Connection 0 Data Bus
		GPIO1_CLKIN,					//	GPIO Connection 1 Clock In Bus
		GPIO1_CLKOUT,					//	GPIO Connection 1 Clock Out Bus
		GPIO1_D							//	GPIO Connection 1 Data Bus
	);

	////////////////////////	Clock Input	 	////////////////////////
input			CLOCK_50;				//	50 MHz
input			CLOCK_50_2;				//	50 MHz
////////////////////////	Push Button		////////////////////////
input	[2:0]	BUTTON;					//	Pushbutton[2:0]
////////////////////////	DPDT Switch		////////////////////////
input	[9:0]	SW;						//	Toggle Switch[9:0]
////////////////////////	7-SEG Dispaly	////////////////////////
output	[6:0]	HEX0_D;					//	Seven Segment Digit 0
output			HEX0_DP;				//	Seven Segment Digit DP 0
output	[6:0]	HEX1_D;					//	Seven Segment Digit 1
output			HEX1_DP;				//	Seven Segment Digit DP 1
output	[6:0]	HEX2_D;					//	Seven Segment Digit 2
output			HEX2_DP;				//	Seven Segment Digit DP 2
output	[6:0]	HEX3_D;					//	Seven Segment Digit 3
output			HEX3_DP;				//	Seven Segment Digit DP 3
////////////////////////////	LED		////////////////////////////
output	[9:0]	LEDG;					//	LED Green[9:0]
////////////////////////////	UART	////////////////////////////
output			UART_TXD;				//	UART Transmitter
input			UART_RXD;				//	UART Receiver
output			UART_CTS;				//	UART Clear To Send
input			UART_RTS;				//	UART Request To Send
///////////////////////		SDRAM Interface	////////////////////////
inout	[15:0]	DRAM_DQ;				//	SDRAM Data bus 16 Bits
output	[12:0]	DRAM_ADDR;				//	SDRAM Address bus 13 Bits
output			DRAM_LDQM;				//	SDRAM Low-byte Data Mask
output			DRAM_UDQM;				//	SDRAM High-byte Data Mask
output			DRAM_WE_N;				//	SDRAM Write Enable
output			DRAM_CAS_N;				//	SDRAM Column Address Strobe
output			DRAM_RAS_N;				//	SDRAM Row Address Strobe
output			DRAM_CS_N;				//	SDRAM Chip Select
output			DRAM_BA_0;				//	SDRAM Bank Address 0
output			DRAM_BA_1;				//	SDRAM Bank Address 1
output			DRAM_CLK;				//	SDRAM Clock
output			DRAM_CKE;				//	SDRAM Clock Enable
////////////////////////	Flash Interface	////////////////////////
inout	[14:0]	FL_DQ;					//	FLASH Data bus 15 Bits
inout			FL_DQ15_AM1;			//	FLASH Data bus Bit 15 or Address A-1
output	[21:0]	FL_ADDR;				//	FLASH Address bus 22 Bits
output			FL_WE_N;				//	FLASH Write Enable
output			FL_RST_N;				//	FLASH Reset
output			FL_OE_N;				//	FLASH Output Enable
output			FL_CE_N;				//	FLASH Chip Enable
output			FL_WP_N;				//	FLASH Hardware Write Protect
output			FL_BYTE_N;				//	FLASH Selects 8/16-bit mode
input			FL_RY;					//	FLASH Ready/Busy
////////////////////	LCD Module 16X2	////////////////////////////
inout	[7:0]	LCD_DATA;				//	LCD Data bus 8 bits
output			LCD_BLON;				//	LCD Back Light ON/OFF
output			LCD_RW;					//	LCD Read/Write Select, 0 = Write, 1 = Read
output			LCD_EN;					//	LCD Enable
output			LCD_RS;					//	LCD Command/Data Select, 0 = Command, 1 = Data
////////////////////	SD Card Interface	////////////////////////
inout			SD_DAT0;				//	SD Card Data 0
inout			SD_DAT3;				//	SD Card Data 3
inout			SD_CMD;					//	SD Card Command Signal
output			SD_CLK;					//	SD Card Clock
input			SD_WP_N;				//	SD Card Write Protect
////////////////////////	PS2		////////////////////////////////
inout		 	PS2_KBDAT;				//	PS2 Keyboard Data
inout			PS2_KBCLK;				//	PS2 Keyboard Clock
inout		 	PS2_MSDAT;				//	PS2 Mouse Data
inout			PS2_MSCLK;				//	PS2 Mouse Clock
////////////////////////	VGA			////////////////////////////
output			VGA_HS;					//	VGA H_SYNC
output			VGA_VS;					//	VGA V_SYNC
output	[3:0]	VGA_R;   				//	VGA Red[3:0]
output	[3:0]	VGA_G;	 				//	VGA Green[3:0]
output	[3:0]	VGA_B;   				//	VGA Blue[3:0]
////////////////////////	GPIO	////////////////////////////////
input	[1:0]	GPIO0_CLKIN;			//	GPIO Connection 0 Clock In Bus
output	[1:0]	GPIO0_CLKOUT;			//	GPIO Connection 0 Clock Out Bus
inout	[31:0]	GPIO0_D;				//	GPIO Connection 0 Data Bus
input	[1:0]	GPIO1_CLKIN;			//	GPIO Connection 1 Clock In Bus
output	[1:0]	GPIO1_CLKOUT;			//	GPIO Connection 1 Clock Out Bus
inout	[31:0]	GPIO1_D;				//	GPIO Connection 1 Data Bus

//=======================================================
//  REG/WIRE declarations
//=======================================================
//	All inout port turn to tri-state
assign			DRAM_DQ			=	16'hzzzz;
assign			FL_DQ			=	16'hzzzz;
assign			LCD_DATA		=	8'hzz;
assign			SD_DAT0			=	1'hz;
assign			SD_DAT3			=	1'hz;
assign			SD_CMD			=	1'hz;
assign			PS2_KBDAT		=	1'hz;
assign			PS2_KBCLK		=	1'hz;
assign			PS2_MSDAT		=	1'hz;
assign			PS2_MSCLK		=	1'hz;
assign			GPIO0_D			=	32'hzzzzzzzz;
assign			GPIO1_D			=	32'hzzzzzzzz;

assign			HEX0_D			=	7'h7F;
assign			HEX0_DP			=	1'h1;
assign			HEX1_D			=	7'h7F;
assign			HEX1_DP			=	1'h1;
assign			HEX2_D			=	7'h7F;
assign			HEX2_DP			=	1'h1;
assign			HEX3_D			=	7'h7F;
assign			HEX3_DP			=	1'h1;

assign			LEDG			=	10'h000;
////////////////////////	VGA			////////////////////////////
wire			VGA_CTRL_CLK;
wire	[9:0]	mVGA_X;
wire	[9:0]	mVGA_Y;
wire	[9:0]	mVGA_R;
wire	[9:0]	mVGA_G;
wire	[9:0]	mVGA_B;
//wire	[19:0]	mVGA_ADDR;
wire	[9:0]	sVGA_R;
wire	[9:0]	sVGA_G;
wire	[9:0]	sVGA_B;
assign	VGA_R	=	sVGA_R[3:0];
assign	VGA_G	=	sVGA_G[3:0];
assign	VGA_B	=	sVGA_B[3:0];

sdram_pll sdram_clocker(
	CLOCK_50,
	DRAM_CLK);

wire sdr_init_done;
wire RESETN;
//--------------------------------------
// Wish Bone Interface
// -------------------------------------      
parameter      dw              = 32;  // data width
parameter      tw              = 8;   // tag id width
parameter      bl              = 9;   // burst_lenght_width 
wire                   wb_rst_i           ;
wire                   wb_clk_i           ;

wire                   wb_stb_i           ;
wire                  wb_ack_o           ;
wire [24:0]            wb_addr_i          ;
wire                   wb_we_i            ; // 1 - Write , 0 - Read
wire [dw-1:0]          wb_dat_i           ;
wire [dw/8-1:0]        wb_sel_i           ; // Byte enable
wire [dw-1:0]         wb_dat_o           ;
wire                   wb_cyc_i           ;
wire  [2:0]            wb_cti_i           ;

sdrc_top u_dut(
      // System 
          .cfg_sdr_width      (2'b01              ), // 16 BIT SDRAM
          .cfg_colbits        (2'b00              ), // 8 Bit Column Address

/* WISH BONE */
          .wb_rst_i           (!SD_WP_N            ),
          .wb_clk_i           (CLOCK_50            ),

          .wb_stb_i           (wb_stb_i           ),
          .wb_ack_o           (wb_ack_o           ),
          .wb_addr_i          (wb_addr_i          ),
          .wb_we_i            (wb_we_i            ),
          .wb_dat_i           (wb_dat_i           ),
          .wb_sel_i           (wb_sel_i           ),
          .wb_dat_o           (wb_dat_o           ),
          .wb_cyc_i           (wb_cyc_i           ),
          .wb_cti_i           (wb_cti_i           ), 

/* Interface to SDRAMs */
          .sdram_clk          (DRAM_CLK          ),
          .sdram_resetn       (SD_WP_N             ),
          .sdr_cs_n           (DRAM_CS_N           ),
          .sdr_cke            (DRAM_CKE            ),
          .sdr_ras_n          (DRAM_RAS_N          ),
          .sdr_cas_n          (DRAM_CAS_N          ),
          .sdr_we_n           (DRAM_WE_N           ),
          .sdr_dqm            (DRAM_LDQM           ),
          .sdr_ba             (DRAM_BA_            ),
          .sdr_addr           (DRAM_ADDR           ), 
          .sdr_dq             (DRAM_DQ             ),

    /* Parameters */
          .sdr_init_done      (sdr_init_done      ),
          .cfg_req_depth      (2'h3               ),	        //how many req. buffer should hold
          .cfg_sdr_en         (1'b1               ),
          .cfg_sdr_mode_reg   (12'h033            ),
          .cfg_sdr_tras_d     (4'h4               ),
          .cfg_sdr_trp_d      (4'h2               ),
          .cfg_sdr_trcd_d     (4'h2               ),
          .cfg_sdr_cas        (3'h3               ),
          .cfg_sdr_trcar_d    (4'h7               ),
          .cfg_sdr_twr_d      (4'h1               ),
          .cfg_sdr_rfsh       (12'h100            ), // reduced from 12'hC35
          .cfg_sdr_rfmax      (3'h6               )

);


	

endmodule
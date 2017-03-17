// Single Cycle MIPS
//=========================================================
// Input/Output Signals:
// positive-edge triggered         CLK
// active low asynchronous reset   RST
// instruction memory interface    IR_addr, IR
// output for testing purposes     RF_writedata  
//=========================================================
// Wire/Reg Specifications:
// control signals             MemToReg, MemRead, MemWrite, 
//                             RegDST, RegWrite, Branch, 
//                             Jump, ALUSrc, ALUOp
// ALU control signals         ALUctrl
// ALU input signals           ALUin1, ALUin2
// ALU output signals          ALUresult, ALUzero
// instruction specifications  r, j, jal, jr, lw, sw, beq
// sign-extended signal        SignExtend
// MUX output signals          MUX_RegDST, MUX_MemToReg, 
//                             MUX_Src, MUX_Branch, MUX_Jump
// registers input signals     Reg_R1, Reg_R2, Reg_W, WriteData 
// registers                   Register
// registers output signals    ReadData1, ReadData2
// data memory contral signals CEN, OEN, WEN
// data memory output signals  ReadDataMem
// program counter/address     PCin, PCnext, JumpAddr, BranchAddr
//=========================================================
module SingleCycleMIPS( 
    CLK,
    RST,
    IR_addr,
    IR,
    RF_writedata,
    ReadDataMem,
    CEN,
    WEN,
    A,
    ReadData2,
    OEN
);

//==== in/out declaration =================================
    //-------- processor ----------------------------------
input           CLK, RST;
input   [31:0]  IR;
output  [31:0]  IR_addr, RF_writedata;
    //-------- data memory --------------------------------
input   [31:0]  ReadDataMem;
output          CEN;
output          WEN;
output  [ 6:0]  A;
output  [31:0]  ReadData2;
output          OEN;

//==== reg/wire declaration ===============================


wire            MemToReg,
                MemRead,
                MemWrite,
                RegDST,
                RegWrite,
                Branch,
                Jump,
                ALUSrc,
                JumpRegister,
                JumpAndLink;
wire    [ 1:0]  ALUOp;
wire    [ 3:0]  ALUctrl;
wire    [31:0]  ALUin1, ALUin2;
wire    [31:0]  ALUresult;
wire            ALUzero;
wire    [31:0]  SignExtend;
wire    [ 4:0]  MUX_RegDST;
wire    [31:0]  MUX_MemToReg;
wire    [31:0]  MUX_Src;
wire    [31:0]  MUX_Branch;
wire    [31:0]  MUX_Jump;
wire    [ 4:0]  Reg_R1, Reg_R2, Reg_W;
wire    [31:0]  WriteData;
wire    [31:0]  ReadData1; // ReadData2 is declared as output
wire    [31:0]  PCin; // PCin update only at posedge CLK
wire    [31:0]  PCnext;
wire    [31:0]  PCAdd4;
wire    [31:0]  JumpAddr, BranchAddr;

//==== module connection ==================================

assign RF_writedata = WriteData; // debugging purpose

assign PCin = PCnext;
PC PC(
  .PC_i  (PCin),
  .PC_o  (IR_addr),
  .CLK   (CLK),
  .RST   (RST)
);

Control Control(
  .opcode_i       (IR[31:26]),
  .funct_i        (IR[5:0]),
  .RegDst_o       (RegDST),
  .Jump_o         (Jump), 
  .Branch_o       (Branch),
  .MemRead_o      (MemRead),
  .MemtoReg_o     (MemToReg),
  .ALUOp_o        (ALUOp),
  .MemWrite_o     (MemWrite),
  .ALUSrc_o       (ALUSrc),
  .RegWrite_o     (RegWrite),
  .JumpRegister_o (JumpRegister),
  .JumpAndLink_o  (JumpAndLink)
);

assign ALUin1 = ReadData1;
assign ALUin2 = MUX_Src;
assign A = ALUresult[8:2]; // A = LSB of (ALUresult >> 2)
ALU ALU(
  .data1_i       (ALUin1),
  .data2_i       (ALUin2),
  .ALUCtrl_i     (ALUctrl),
  .ALUResult_o   (ALUresult),
  .Zero_o        (ALUzero)
);

ALUControl ALUControl(
  .ALUOp_i    (ALUOp),
  .funct_i    (IR[5:0]),
  .ALUCtrl_o  (ALUctrl)
);

SignExtendUnit SignExtendUnit(
  .data_i  (IR[15:0]),
  .data_o  (SignExtend)
);

assign Reg_R1 = IR[25:21];
assign Reg_R2 = IR[20:16];
Registers Register(
  .Rreg1_i    (Reg_R1),
  .Rreg2_i    (Reg_R2),
  .Wreg_i     (Reg_W),
  .Wdata_i    (WriteData),
  .RegWrite_i (RegWrite),
  .Rdata1_o   (ReadData1),
  .Rdata2_o   (ReadData2),
  .CLK        (CLK),
  .RST        (RST)
);

MUX2 #(5) MUX2_RegDST(
  .data1_i    (IR[20:16]),
  .data2_i    (IR[15:11]),
  .select_i   (RegDST),
  .data_o     (MUX_RegDST)
);

MUX2 #(32) MUX2_Src(
  .data1_i    (ReadData2),
  .data2_i    (SignExtend),
  .select_i   (ALUSrc),
  .data_o     (MUX_Src)
);

MUX2 #(32) MUX2_MemToReg(
  .data1_i    (ALUresult),
  .data2_i    (ReadDataMem),
  .select_i   (MemToReg),
  .data_o     (MUX_MemToReg)
);

Adder PCAdder(
  .data1_i    (IR_addr),
  .data2_i    (32'b100),
  .result_o   (PCAdd4)
);

assign JumpAddr = { {PCAdd4[31:28]}, {IR[25:0]}, 2'b00};

Adder BranchAdder(
  .data1_i    (PCAdd4),
  .data2_i    ({{SignExtend[29:0]}, 2'b00}),
  .result_o   (BranchAddr)
);

MUX2 #(32) MUX2_Branch(
  .data1_i    (PCAdd4),
  .data2_i    (BranchAddr),
  .select_i   (Branch & ALUzero),
  .data_o     (MUX_Branch)
);

MUX2 #(32) MUX2_Jump(
  .data1_i    (MUX_Branch),
  .data2_i    (JumpAddr),
  .select_i   (Jump),
  .data_o     (MUX_Jump)
);

MUX2 #(32) MUX2_JumpRegister(
  .data1_i    (MUX_Jump),
  .data2_i    (ReadData1),
  .select_i   (JumpRegister),
  .data_o     (PCnext)
);

MUX2 #(5) MUX2_JumpAndLink_1(
  .data1_i    (MUX_RegDST),
  .data2_i    (5'd31),
  .select_i   (JumpAndLink),
  .data_o     (Reg_W)
);

MUX2 #(32) MUX2_JumpAndLink_2(
  .data1_i    (MUX_MemToReg),
  .data2_i    (PCAdd4), // NOT PCAdd8
  .select_i   (JumpAndLink),
  .data_o     (WriteData)
);
//assign PCnext = MUX_Jump;

assign CEN = ~(MemRead | MemWrite);
assign WEN = ~(MemWrite);
assign OEN = ~(MemRead);

//==== combinational part =================================
always @ (*) begin
    
end

//==== sequential part ====================================
//always @ (posedge CLK or posedge RST) begin
//
//end

//=========================================================
endmodule


// Program Counter (PC)
module PC(PC_i, PC_o, CLK, RST);

input               CLK;
input               RST;
input      [31:0]   PC_i;
output     [31:0]   PC_o;

reg        [31:0]   PC_buffer;

assign PC_o = PC_buffer;

always @ (posedge CLK or posedge RST) begin
  if(RST)
    PC_buffer <= 32'b0;
  else
    PC_buffer <= PC_i;
end

endmodule


// Adder
module Adder(data1_i, data2_i, result_o);

input   [31:0]   data1_i;
input   [31:0]   data2_i;
output  [31:0]   result_o;

assign result_o = data1_i + data2_i;

endmodule


// Registers
module Registers(Rreg1_i, Rreg2_i, Wreg_i, Wdata_i, RegWrite_i, Rdata1_o, Rdata2_o, CLK, RST);

input       [4:0]   Rreg1_i;
input       [4:0]   Rreg2_i;
input       [4:0]   Wreg_i;
input       [31:0]  Wdata_i;
input               RegWrite_i;
output      [31:0]  Rdata1_o;
output      [31:0]  Rdata2_o;
input               CLK;
input               RST;

reg         [31:0]  register  [0:31]; // 32 32-bits registers
integer             i; // for iteration purpose

assign Rdata1_o = register[Rreg1_i];
assign Rdata2_o = register[Rreg2_i];

// Write data ( sequential )
always @ (posedge CLK or posedge RST) begin
  if(RST) begin
    for(i = 0; i < 32; i = i + 1) begin
      register[i] <= 32'b0;
    end
  end
  else if(RegWrite_i) begin
    register[Wreg_i] <= Wdata_i;
  end
end

endmodule


// Control
module Control( opcode_i,
                funct_i,
                RegDst_o, 
                Jump_o, 
                Branch_o,
                MemRead_o,
                MemtoReg_o,
                ALUOp_o,
                MemWrite_o,
                ALUSrc_o,
                RegWrite_o,
                JumpRegister_o,
                JumpAndLink_o);

input  [5:0]  opcode_i;
input  [5:0]  funct_i;
output        RegDst_o;
output        Jump_o;
output        Branch_o;
output        MemRead_o;
output        MemtoReg_o;
output [1:0]  ALUOp_o;
output        MemWrite_o;
output        ALUSrc_o;
output        RegWrite_o;
output        JumpRegister_o;
output        JumpAndLink_o;

wire R_format, lw, sw, beq, jump, jr, jal;

assign R_format = (opcode_i == 6'b000000) ? 1 : 0;
assign lw       = (opcode_i == 6'b100011) ? 1 : 0;
assign sw       = (opcode_i == 6'b101011) ? 1 : 0;
assign beq      = (opcode_i == 6'b000100) ? 1 : 0;
assign jump     = (opcode_i == 6'b000010) ? 1 : 0;
assign jr       = ((opcode_i == 6'b000000) && (funct_i == 6'b001000)) ? 1 : 0;
assign jal      = (opcode_i == 6'b000011) ? 1 : 0;


assign RegDst_o   = R_format;
assign Jump_o     = jump | jal;
assign Branch_o   = beq;
assign MemRead_o  = lw;
assign MemtoReg_o = lw;
assign MemWrite_o = sw;
assign ALUSrc_o   = lw | sw;
assign RegWrite_o = R_format | lw | jal;
assign ALUOp_o    = R_format ? 2'b10 :
                    beq      ? 2'b01 : 2'b00 ; // Default and lw / sw
assign JumpRegister_o = jr;
assign JumpAndLink_o  = jal;


endmodule



// ALU
module ALU(data1_i, data2_i, ALUCtrl_i, ALUResult_o, Zero_o);

input signed  [31:0] data1_i;
input signed  [31:0] data2_i;
input  [ 3:0] ALUCtrl_i;  
output reg signed [31:0] ALUResult_o;
output        Zero_o; // 1 bit for branch control

assign Zero_o = (data1_i==data2_i) ? 1 : 0;

// ALU control MACRO for different operation
localparam ADD = 4'b0010,
           SUB = 4'b0110,
           AND = 4'b0000,
           OR  = 4'b0001,
           SLT = 4'b0111;

always @ (*) begin
  case (ALUCtrl_i)
    ADD: ALUResult_o = data1_i + data2_i;
    SUB: ALUResult_o = data1_i - data2_i;
    AND: ALUResult_o = data1_i & data2_i;
    OR : ALUResult_o = data1_i | data2_i;
    SLT: ALUResult_o = (data1_i < data2_i) ? 32'b1 : 32'b0; 
    default: ALUResult_o = 32'b0;
  endcase
end

endmodule


// ALU Control for ALU
module ALUControl(ALUOp_i, funct_i, ALUCtrl_o);

input  [1:0]  ALUOp_i;
input  [5:0]  funct_i;
output [3:0]  ALUCtrl_o;

// ALU control MACRO for different operation
localparam ADD = 4'b0010,
           SUB = 4'b0110,
           AND = 4'b0000,
           OR  = 4'b0001,
           SLT = 4'b0111;

assign ALUCtrl_o =
  ALUOp_i == 2'b10 ?
    (
      funct_i[3:0] == 4'b0000 ? ADD :
      funct_i[3:0] == 4'b0010 ? SUB :
      funct_i[3:0] == 4'b0100 ? AND :
      funct_i[3:0] == 4'b0101 ? OR  :
      funct_i[3:0] == 4'b1010 ? SLT : 4'b0000
    ) :
    ALUOp_i == 2'b00 ? ADD :
    ALUOp_i == 2'b01 ? SUB : 4'b0000; // Default 4'b0000

endmodule


// Sign Extend: extend 16-bits input to 32-bits output
module SignExtendUnit(data_i, data_o);

input   [15:0]  data_i;
output  [31:0]  data_o;

assign data_o = { {16{data_i[15]}} , data_i };

endmodule


// 32-bits 2-to-1 MUX with 1-bit select
module MUX2 #(parameter WIDTH=32)( data1_i, data2_i, select_i, data_o);

input   [WIDTH-1:0]  data1_i;
input   [WIDTH-1:0]  data2_i;
input                select_i;
output  [WIDTH-1:0]  data_o;

assign data_o = select_i ? data2_i : data1_i;

endmodule




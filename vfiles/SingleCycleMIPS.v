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
output  [6:0]   A;
output  [31:0]  ReadData2;
output          OEN;

//==== reg/wire declaration ===============================


//==== combinational part =================================
always @ (*) begin
    
end

//==== sequential part ====================================
always @ (posedge CLK or posedge RST) begin
    
end

//=========================================================
endmodule

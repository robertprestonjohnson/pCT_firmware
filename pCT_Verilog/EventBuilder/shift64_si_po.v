//64 bit register serial in, parallel out, with synchronous reset

module shift64_si_po (Clk, Reset, Load, Si, Po);
input Clk;           //Clock to shift the mask setting in, always active
input Reset;         //Synchronous reset to default values when high       
input Load;          //Serial load of register when high; hold when low
input Si;            //Serial input for loading the register
output [63:0] Po;    //Parallel output of the register

wire Clk, Reset, Load, Si;
wire [63:0] Po;

ShiftCell ShiftCell0(Clk, Reset, 1'b1, Load, Si, Po[0]);
ShiftCell ShiftCell1(Clk, Reset, 1'b1, Load, Po[0], Po[1]);
ShiftCell ShiftCell2(Clk, Reset, 1'b1, Load, Po[1], Po[2]);
ShiftCell ShiftCell3(Clk, Reset, 1'b1, Load, Po[2], Po[3]);
ShiftCell ShiftCell4(Clk, Reset, 1'b1, Load, Po[3], Po[4]);
ShiftCell ShiftCell5(Clk, Reset, 1'b1, Load, Po[4], Po[5]);
ShiftCell ShiftCell6(Clk, Reset, 1'b1, Load, Po[5], Po[6]);
ShiftCell ShiftCell7(Clk, Reset, 1'b1, Load, Po[6], Po[7]);
ShiftCell ShiftCell8(Clk, Reset, 1'b1, Load, Po[7], Po[8]);
ShiftCell ShiftCell9(Clk, Reset, 1'b1, Load, Po[8], Po[9]);
ShiftCell ShiftCell10(Clk, Reset, 1'b1, Load, Po[9], Po[10]);
ShiftCell ShiftCell11(Clk, Reset, 1'b1, Load, Po[10], Po[11]);
ShiftCell ShiftCell12(Clk, Reset, 1'b1, Load, Po[11], Po[12]);
ShiftCell ShiftCell13(Clk, Reset, 1'b1, Load, Po[12], Po[13]);
ShiftCell ShiftCell14(Clk, Reset, 1'b1, Load, Po[13], Po[14]);
ShiftCell ShiftCell15(Clk, Reset, 1'b1, Load, Po[14], Po[15]);
ShiftCell ShiftCell16(Clk, Reset, 1'b1, Load, Po[15], Po[16]);
ShiftCell ShiftCell17(Clk, Reset, 1'b1, Load, Po[16], Po[17]);
ShiftCell ShiftCell18(Clk, Reset, 1'b1, Load, Po[17], Po[18]);
ShiftCell ShiftCell19(Clk, Reset, 1'b1, Load, Po[18], Po[19]);
ShiftCell ShiftCell20(Clk, Reset, 1'b1, Load, Po[19], Po[20]);
ShiftCell ShiftCell21(Clk, Reset, 1'b1, Load, Po[20], Po[21]);
ShiftCell ShiftCell22(Clk, Reset, 1'b1, Load, Po[21], Po[22]);
ShiftCell ShiftCell23(Clk, Reset, 1'b1, Load, Po[22], Po[23]);
ShiftCell ShiftCell24(Clk, Reset, 1'b1, Load, Po[23], Po[24]);
ShiftCell ShiftCell25(Clk, Reset, 1'b1, Load, Po[24], Po[25]);
ShiftCell ShiftCell26(Clk, Reset, 1'b1, Load, Po[25], Po[26]);
ShiftCell ShiftCell27(Clk, Reset, 1'b1, Load, Po[26], Po[27]);
ShiftCell ShiftCell28(Clk, Reset, 1'b1, Load, Po[27], Po[28]);
ShiftCell ShiftCell29(Clk, Reset, 1'b1, Load, Po[28], Po[29]);
ShiftCell ShiftCell30(Clk, Reset, 1'b1, Load, Po[29], Po[30]);
ShiftCell ShiftCell31(Clk, Reset, 1'b1, Load, Po[30], Po[31]);
ShiftCell ShiftCell32(Clk, Reset, 1'b1, Load, Po[31], Po[32]);
ShiftCell ShiftCell33(Clk, Reset, 1'b1, Load, Po[32], Po[33]);
ShiftCell ShiftCell34(Clk, Reset, 1'b1, Load, Po[33], Po[34]);
ShiftCell ShiftCell35(Clk, Reset, 1'b1, Load, Po[34], Po[35]);
ShiftCell ShiftCell36(Clk, Reset, 1'b1, Load, Po[35], Po[36]);
ShiftCell ShiftCell37(Clk, Reset, 1'b1, Load, Po[36], Po[37]);
ShiftCell ShiftCell38(Clk, Reset, 1'b1, Load, Po[37], Po[38]);
ShiftCell ShiftCell39(Clk, Reset, 1'b1, Load, Po[38], Po[39]);
ShiftCell ShiftCell40(Clk, Reset, 1'b1, Load, Po[39], Po[40]);
ShiftCell ShiftCell41(Clk, Reset, 1'b1, Load, Po[40], Po[41]);
ShiftCell ShiftCell42(Clk, Reset, 1'b1, Load, Po[41], Po[42]);
ShiftCell ShiftCell43(Clk, Reset, 1'b1, Load, Po[42], Po[43]);
ShiftCell ShiftCell44(Clk, Reset, 1'b1, Load, Po[43], Po[44]);
ShiftCell ShiftCell45(Clk, Reset, 1'b1, Load, Po[44], Po[45]);
ShiftCell ShiftCell46(Clk, Reset, 1'b1, Load, Po[45], Po[46]);
ShiftCell ShiftCell47(Clk, Reset, 1'b1, Load, Po[46], Po[47]);
ShiftCell ShiftCell48(Clk, Reset, 1'b1, Load, Po[47], Po[48]);
ShiftCell ShiftCell49(Clk, Reset, 1'b1, Load, Po[48], Po[49]);
ShiftCell ShiftCell50(Clk, Reset, 1'b1, Load, Po[49], Po[50]);
ShiftCell ShiftCell51(Clk, Reset, 1'b1, Load, Po[50], Po[51]);
ShiftCell ShiftCell52(Clk, Reset, 1'b1, Load, Po[51], Po[52]);
ShiftCell ShiftCell53(Clk, Reset, 1'b1, Load, Po[52], Po[53]);
ShiftCell ShiftCell54(Clk, Reset, 1'b1, Load, Po[53], Po[54]);
ShiftCell ShiftCell55(Clk, Reset, 1'b1, Load, Po[54], Po[55]);
ShiftCell ShiftCell56(Clk, Reset, 1'b1, Load, Po[55], Po[56]);
ShiftCell ShiftCell57(Clk, Reset, 1'b1, Load, Po[56], Po[57]);
ShiftCell ShiftCell58(Clk, Reset, 1'b1, Load, Po[57], Po[58]);
ShiftCell ShiftCell59(Clk, Reset, 1'b1, Load, Po[58], Po[59]);
ShiftCell ShiftCell60(Clk, Reset, 1'b1, Load, Po[59], Po[60]);
ShiftCell ShiftCell61(Clk, Reset, 1'b1, Load, Po[60], Po[61]);
ShiftCell ShiftCell62(Clk, Reset, 1'b1, Load, Po[61], Po[62]);
ShiftCell ShiftCell63(Clk, Reset, 1'b1, Load, Po[62], Po[63]);

endmodule


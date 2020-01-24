//New program to receive data from the pCT tracker ASICs
//R. Johnson  April 8, 2014

module TkrDatReceive(BufClr0,BufClr1,BufClr2,BufClr3,nFull,PrtyErrOut,Error,DataOut,Strobe,Send,DataIn,Clock,Reset,Chip,Address);
input [3:0] Address;	//FPGA address (just to control debug printing)
input [3:0] Chip;		//ID of the chip being handled (0-11)
input Clock;
input Reset;
input DataIn;			//Input serial stream from the ASIC
input Send;				//Signal to send out the first word of the next event
input Strobe;			//Send out the next word of the event
output [11:0] DataOut;	//Output data word
output Error;			//Pulses each time an error is encountered
output PrtyErrOut;		//Indicates whether the parity of the current event going out was correct
output [3:0] nFull;		//Number of front-end board FPGA buffers full
output BufClr0, BufClr1, BufClr2, BufClr3;	//Pulses each time an ASIC has finished sending its data for a given tag

reg [11:0] DataOut;
reg Error;
reg [3:0] BufClr;
assign BufClr0 = BufClr[0];
assign BufClr1 = BufClr[1];
assign BufClr2 = BufClr[2];
assign BufClr3 = BufClr[3];

reg [2:0] writePtr, readPtr;   //Write and read pointers to select one of the 8 RAM buffers

//State machine to send one event out
parameter [2:0] WaitSd = 3'b001;   //Wait for a start signal
parameter [2:0] NextSd = 3'b010;   //Read the next data word from memory
parameter [2:0] DoneSd = 3'b100;   //Signal that the event is done
reg[2:0] StateSd, NextStateSd;
reg[3:0] NWrds;						//Number of words for the event being read

reg First;
assign Done = (StateSd == DoneSd && First);
always @ (StateSd or Send or nFull or readPtr or NWrds or readAddr or Strobe) begin
	case (StateSd) 
		WaitSd: begin
					if (Send) begin
						readEnable = ThreeToEight(readPtr);
						if (NWrds == 1) NextStateSd = DoneSd;
						else NextStateSd = NextSd;
					end else begin
						NextStateSd = WaitSd;
						readEnable = 1'b0;
					end
				end
		NextSd:	begin
					if (readAddr >= NWrds-1) begin
						if (Strobe) NextStateSd = DoneSd;
						else NextStateSd = NextSd;
					end else NextStateSd = NextSd;
					if (Strobe) readEnable = ThreeToEight(readPtr);
					else readEnable = 1'b0;
				end
		DoneSd:	begin
					if (Strobe) NextStateSd = WaitSd;
					else NextStateSd = DoneSd;
					readEnable = 1'b0;
				end
		default: begin NextStateSd = WaitSd; readEnable = 1'b0; end
	endcase
end

assign PrtyErrOut = PrtyErr[readPtr];
always @ (posedge Clock)
begin
	if (Reset) begin
		StateSd <= WaitSd;
		readPtr <= 0;
		readAddr <= 0;
	end else begin
		StateSd <= NextStateSd;
		// if (Address == 0 && Chip == 4 && (StateSd != WaitSd || Send==1'b1)) begin
			// $display("%g\t TkrDatReceive %d %d: StateSd=%b NWrds=%d readAddr=%d readPtr=%d Strobe=%b readEnable=%b DataOut=%b Parity=%b Prty=%b",$time,Address,Chip,StateSd,NWrds,readAddr,readPtr,Strobe,readEnable,DataOut,Parity,Prty);
		// end
		case (StateSd)
			WaitSd:	begin
						NWrds <= nWordsMem[readPtr];
						if (Send) begin
							readAddr <= readAddr + 1;
						end
						First <= 1'b1;
					end
			NextSd:	begin
						if (Strobe) readAddr <= readAddr + 1;
					end
			DoneSd:	begin
						First <= 1'b0;
						if (Strobe) begin
							readPtr <= readPtr + 1;
							readAddr <= 0;
						end
					end
		endcase
	end
end

//State machine to parse the incoming serial data and store it in RAM
parameter [4:0] Wait=5'b00001;		//Wait for the start bit
parameter [4:0] Hddr=5'b00010;		//Shift in the 12 bit header word
parameter [4:0] Regs=5'b00100;		//Skip over a register dump
parameter [4:0] Clus=5'b01000;		//Shift in a 12 bit cluster word
parameter [4:0] EoEv=5'b10000;		//End of the event packet

reg [4:0] State, NextState;

always @ (State or DataIn or Header or CtrH or Ctr or CtrClus or nClus or writePtr) begin
	case (State)
		Wait:	begin
					if (DataIn) NextState = Hddr;
					else NextState = Wait;
					writeEnable = 0;
				end
		Hddr:	begin
					if (CtrH == 10) begin
						if (Header[9] == 1'b1) begin
							NextState = Regs;				//PacketType=1 really should not happen during a run
						end else begin
							if ({Header[2:0],DataIn} == 0) NextState = EoEv; //Zero clusters
							else NextState = Clus;
						end
					end	else NextState = Hddr;
					writeEnable = 0;
				end
		Regs: 	begin
					if (Ctr==66) NextState = Wait;
					else NextState = Regs;
					writeEnable = 0;
				end
		Clus:	begin
					if (Ctr == 0) begin
						writeEnable = ThreeToEight(writePtr);
					end else begin
						writeEnable = 0;
					end
					if (Ctr == 11 && CtrClus == nClus - 1) NextState = EoEv;
					else NextState = Clus;
				end
		EoEv:	begin
					NextState = Wait;
					writeEnable = ThreeToEight(writePtr);
				end
		default:	begin
						NextState = Wait;
						writeEnable = 0;
					end
	endcase
end

reg [3:0] CtrH, CtrClus;
reg [6:0] Ctr;
reg Prty;
assign packetType = Header[10];
wire [3:0] nClus;
wire [1:0] TrigTag;
assign nClus = Header[3:0];
assign Parity = Header[6];
assign TrigTag = Header[9:8];

reg [11:0] Header;
reg [3:0] nFull;
always @ (posedge Clock) begin
	if (Reset) begin
		State <= Wait;
		writePtr <= 0;
		nFull <= 0;
	end else begin
		State <= NextState;
		// if ((State != Wait || DataIn==1'b1) && Address == 0 && Chip == 4) begin
			// $display("%g\t TkrDatReceive %d %d: State=%b writePtr=%d writeAddr=%d nFull=%d dIn=%b DataIn=%b CtrH=%d Ctr=%d CtrClus=%d nClus=%d writeEnable=%b",$time,Address,Chip,State,writePtr,writeAddr,nFull,dIn,DataIn,CtrH,Ctr,CtrClus,nClus,writeEnable);
		// end
		case (State)
			Wait:	begin
						CtrH <= 0;
						dIn[0] <= DataIn;
						Header[0] <= DataIn;
						if (Done) begin
							nFull <= nFull - 1;
//							$display("%g\t TkrDatReceive %d %d: decrementing nFull to %d",$time,Address,Chip,nFull-1);
							if (nFull == 0) begin
								Error <= 1'b1;
								$display("%g\t TkrDatReceive %d %d state %b: buffer underflow error!",$time,Address,Chip,State);
							end
						end else Error <= 1'b0;
						Prty <= 1'b1;
						BufClr <= 4'b0000;
						writeAddr <= 0;
					end
			Hddr:	begin
						CtrH <= CtrH + 1;
						dIn <= {dIn[10:0],DataIn};
						Header <= {Header[10:0],DataIn};
						CtrClus <= 0;
						Ctr <= 0;
						if (Done) begin
							nFull <= nFull - 1;
//							$display("%g\t TkrDatReceive %d %d: decrementing nFull to %d",$time,Address,Chip,nFull-1);
							if (nFull == 0) begin
								Error <= 1'b1;
								$display("%g\t TkrDatReceive %d %d state %b: buffer underflow error!",$time,Address,Chip,State);
							end
						end else Error <= 1'b0;
						Prty <= Prty^DataIn;
					end
			Regs:	begin
						Ctr <= Ctr + 1;
						if (Ctr == 0) $display("%g\t TkrDatReceive %d %d: Error; register serial stream detected.",$time,Address,Chip);
					end
			Clus:	begin
						dIn <= {dIn[10:0],DataIn};
						if (Ctr == 0) begin
							writeAddr <= writeAddr + 1;
							// if (CtrClus == 0) begin
								// $display("%g\t TkrDatReceive %d %d %d: Header=%b tag=%d Error=%b Number clusters=%d",$time,Address,Chip,writePtr,Header,Header[9:8],Header[7],nClus);
							// end else begin
								// $display("%g\t TkrDatReceive %d %d %d: Cluster=%b nStripM1=%d First=%d",$time,Address,Chip,writePtr,dIn,dIn[11:6],dIn[5:0]);
							// end
						end
						if (Ctr < 11) begin
							Ctr <= Ctr + 1;
						end else begin
							Ctr <= 0;
							CtrClus <= CtrClus + 1;
						end
						if (Done) begin
							nFull <= nFull - 1;
//							$display("%g\t TkrDatReceive %d %d: decrementing nFull to %d",$time,Address,Chip,nFull-1);
							if (nFull == 0) begin
								Error <= 1'b1;
								$display("%g\t TkrDatReceive %d %d state %b: buffer underflow error!",$time,Address,Chip,State);
							end
						end else Error <= 1'b0;
						Prty <= Prty^DataIn;
					end
			EoEv:	begin
						if (nClus>0) $display("%g\t TkrDatReceive %d %d %d: Cluster=%b nStripM1=%d First=%d",$time,Address,Chip,writePtr,dIn,dIn[11:6],dIn[5:0]);
						nWordsMem[writePtr] <= nClus + 1;
						PrtyErr[writePtr] <= Prty;
						writePtr <= writePtr + 1;
						if (!Done) begin
							nFull <= nFull + 1;
//							$display("%g\t TkrDatReceive %d %d: incrementing nFull to %d",$time,Address,Chip,nFull+1);
							if (nFull == 8) begin
								$display("%g\t TkrDatReceive %d %d: buffer overflow error.",$time,Address,Chip);
								Error <= 1'b1;
							end else Error <= packetType;
						end else begin
							Error <= packetType;
							$display("%g\t TkrDatReceive %d %d: simultaneous EoEv and Done, keep nFull=%d",$time,Address,Chip,nFull);
						end
						$display("%g\t TkrDatReceive %d %d: BufClr pulsing for tag %h",$time,Address,Chip,TrigTag);
						BufClr[TrigTag] <= 1'b1;
					end
		endcase
	end
end

reg [7:0] writeEnable, readEnable;
reg [3:0] writeAddr, readAddr;
reg [3:0] nWordsMem[0:7];		//The number of words stored in each RAM buffer
reg [7:0] PrtyErr;				//Set to 1 for a given buffer if a parity error was detected
reg [11:0] dIn;

//Multiplex the 8 RAM outputs onto the single data output line
always @ (readPtr or dOut0 or dOut1 or dOut2 or dOut3 or dOut4 or dOut5 or dOut6 or dOut7) begin
	case (readPtr) 
		3'h0:	DataOut = dOut0;
		3'h1:	DataOut = dOut1;
		3'h2:	DataOut = dOut2;
		3'h3:	DataOut = dOut3;
		3'h4:	DataOut = dOut4;
		3'h5:	DataOut = dOut5;
		3'h6:	DataOut = dOut6;
		3'h7:	DataOut = dOut7;
	endcase
end

wire [11:0] doa0,doa1,doa2,doa3,doa4,doa5,doa6,doa7;
wire [11:0] dOut0, dOut1, dOut2, dOut3, dOut4, dOut5, dOut6, dOut7;
DataRecRAM DataRecRAM0(Clock,Clock,writeEnable[0],readEnable[0],writeEnable[0],writeAddr,readAddr,dIn,doa0,dOut0,3'h0);
DataRecRAM DataRecRAM1(Clock,Clock,writeEnable[1],readEnable[1],writeEnable[1],writeAddr,readAddr,dIn,doa1,dOut1,3'h1);
DataRecRAM DataRecRAM2(Clock,Clock,writeEnable[2],readEnable[2],writeEnable[2],writeAddr,readAddr,dIn,doa2,dOut2,3'h2);
DataRecRAM DataRecRAM3(Clock,Clock,writeEnable[3],readEnable[3],writeEnable[3],writeAddr,readAddr,dIn,doa3,dOut3,3'h3);
DataRecRAM DataRecRAM4(Clock,Clock,writeEnable[4],readEnable[4],writeEnable[4],writeAddr,readAddr,dIn,doa4,dOut4,3'h4);
DataRecRAM DataRecRAM5(Clock,Clock,writeEnable[5],readEnable[5],writeEnable[5],writeAddr,readAddr,dIn,doa5,dOut5,3'h5);
DataRecRAM DataRecRAM6(Clock,Clock,writeEnable[6],readEnable[6],writeEnable[6],writeAddr,readAddr,dIn,doa6,dOut6,3'h6);
DataRecRAM DataRecRAM7(Clock,Clock,writeEnable[7],readEnable[7],writeEnable[7],writeAddr,readAddr,dIn,doa7,dOut7,3'h7);

//3 to 8 decoder
function [7:0] ThreeToEight;
	input [2:0] a;
	case (a)
		3'h0: ThreeToEight = 8'b00000001;
		3'h1: ThreeToEight = 8'b00000010;
		3'h2: ThreeToEight = 8'b00000100;
		3'h3: ThreeToEight = 8'b00001000;
		3'h4: ThreeToEight = 8'b00010000;
		3'h5: ThreeToEight = 8'b00100000;
		3'h6: ThreeToEight = 8'b01000000;
		3'h7: ThreeToEight = 8'b10000000;
	endcase
endfunction

endmodule

//Dual ported RAM with one write port:
module DataRecRAM (clka, clkb, ena, enb, wea, addra, addrb, dia, doa, dob, inst);
input [2:0] inst;
input clka, clkb;
input wea;                  //Write enable
input ena, enb;             //Primary and secondary read enables
input [3:0] addra, addrb;   //Addresses for the primary and secondary ports
input [11:0] dia;           //Input data register to write to memory
output [11:0] doa, dob;     //Output registers for the two ports
reg [11:0] RAM [15:0];     //Memory array
reg [11:0] dob, doa;

always @(posedge clka)
begin
	if (ena)
	begin
		if (wea) begin
			RAM[addra]<=dia;  
			doa <= dia;
//			$display("%g\t DataRecRAM %d: writing data word %b into address %d.",$time,inst,dia,addra);
		end else doa <= RAM[addra];        //Write first, then read
	end
end

always @(posedge clkb)
begin
	if (enb)
	begin
		dob <= RAM[addrb];
//		$display("%g\t DataRecRAM %d: reading data word %b from address %d.",$time,inst,RAM[addrb],addrb);
	end
end
endmodule

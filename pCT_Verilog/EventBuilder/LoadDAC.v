//FPGA code to load the register for the dac101c081 on the energy digitizer board.
//The tri-state buffers for the i2c signals are at the I/O ports of the FPGA.
//Here the incoming and outgoing signals are kept separated.
//This routine assumes that the FPGA operates at 100 MHz.  The i2c clock is
//derived by dividing down by 1024.
//R. Johnson,  February 2013
//July 23, 2013:  changed so that incoming SDA data is latched on the rising edge of SCL instead of SlowClk2
//August 7, 2013: wait 1 more clock cycle to lower SDA at start, and adjust SlowClk2
module LoadDAC(Error,RegOut,StrobeOut,SCL,SCLen,SDAout,SDAen,SDAin,RegIn,Address,StrobeIn,ReadCmd,Clock,RstCmd,Reset);
input Clock;			//Fast (100 MHz) clock from the FPGA
input Reset;			//Reset pulse one clock in length
input RstCmd;			//Pulse to reset the I2C state 
input [6:0] Address;	//i2c address of the DAC chip
output SCL;				//i2c master clock (100 kHz)
output SCLen;			//enable for the SCL output
output SDAout;			//i2c SDA output signal
output SDAen;			//Enable for SDA output tri-state buffer
input SDAin;			//i2c SDA data coming from the DAC
input [15:0] RegIn;		//Input setting for the DAC register from the DAQ
output [15:0] RegOut;	//Output DAC register contents back to the DAQ
input StrobeIn;			//Signal that RegIn is valid and needs to be sent to the DAC
input ReadCmd;			//Signal to read the DAC register contents and load into RegOut;
output StrobeOut;		//Signal that RegOut is valid and can be picked up by the DAQ
output [1:0] Error;		//Error flag:  1= failure to acknowledge address
						//             2= failure to acknowledge first byte
						//             3= failure to acknowledge second byte

reg [15:0] RegOut;
reg SCL, SCLen;
reg SDAout, SDAen;
reg StrobeOut;
reg StrtRead;
reg [8:0] CntClk;
reg [3:0] CntBit;
reg [15:0] RegLatch;
reg [1:0] Error;

reg SlowClk, SlowClk2;
reg StrtSnd, DoneSnd, DoneRead;
reg [15:0] DACreg;		//DAC register contents to shift out to the DAC

reg StateR, NextStateR;
reg [9:0] CntR;
reg ResetI2C;

//State machine to generate a long reset pulse for the i2c state machine.
//This synchronous reset needs to be high a full period of the slow i2c clock.
always @ (StateR or RstCmd or CntR) begin
	case (StateR)
		1'b0:	begin
					if (RstCmd) NextStateR = 1'b1;
					else NextStateR = 1'b0;
				end
		1'b1:	begin
					if (CntR == 1023) NextStateR = 1'b0;
					else NextStateR = 1'b1;
				end
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin
		ResetI2C <= 1'b0;
		StateR <= 1'b0;
	end else begin
		StateR <= NextStateR;
		case (StateR)
			1'b0:	begin
						CntR <= 0;
						ResetI2C <= 1'b0;
					end
			1'b1:	begin
						if (CntR == 0) $display("%g\t LoadDAC:  sending Reset pulse to the i2c state machine.",$time);
						CntR <= CntR + 1;
						ResetI2C <= 1'b1;
					end
		endcase
	end
end

//Fast interface state machine.  Here we grab the signals and data from the
//FPGA program, which is running at 100 MHz.  This machine also generates
//the slow clocks needed to implement the i2c communication.
parameter [3:0] WaitF = 4'b0001;
parameter [3:0] SendF = 4'b0010;
parameter [3:0] ReadF = 4'b0100;

reg [3:0] StateF, NextStateF;

always @ (StateF or StrobeIn or ReadCmd or DoneSnd or DoneRead) begin
	case (StateF)
		WaitF:	begin
					if (StrobeIn) NextStateF = SendF;
					else if (ReadCmd) NextStateF = ReadF;
					else NextStateF = WaitF;
				end
		SendF: 	begin
					if (DoneSnd) NextStateF = WaitF;
					else NextStateF = SendF;
				end
		ReadF:	begin
					if (DoneRead) NextStateF = WaitF;
					else NextStateF = ReadF;
				end
		default: NextStateF = WaitF;
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin
		SlowClk <= 1'b0;
		SlowClk2 <= 1'b0;
		SCL <= 1'b1;
		StateF <= 1'b0;
		CntClk <= 0;
	end else begin
		StateF <= NextStateF;
		CntClk <= CntClk + 1;
		if (CntClk == 511) SlowClk <= ~SlowClk;		//Clock to control i2c data transitions
		if (CntClk == 255) SCL <= ~SCL;             //SCL 180 degrees out of phase w.r.t. data transitions
													//SCL is the i2c "clock".  SDA "data" transitions must only
													//occur while SCL is low, except for start and stop signals.
		if (CntClk == 127) SlowClk2 <= ~SlowClk2;   //Another phased clock, used to enable and disable SCL at the
													//correct times.
		case (StateF)
			WaitF:	begin
						StrtSnd <= 1'b0;
						StrtRead <= 1'b0;
						if (StrobeIn) begin
							RegLatch <= RegIn;
							$display("%g\t LoadDAC:  inputing register data %b",$time,RegIn);
						end
						if (ReadCmd) $display("%g\t LoadDAC:  read command received.",$time);
					end
			SendF:	begin
						StrtSnd <= 1'b1;    //Initiate loading a register over i2c.
					end
			ReadF:	begin
						StrtRead <= 1'b1;   //Initiate reading of a register over i2c.
					end
		endcase
	end
end

//State machine with a slow i2c clock
parameter [15:0] Wait = 16'b0000000000000001;		//Wait for a StrtSnd or StrtRead signal
parameter [15:0] SndR = 16'b0000000000000010;		//Send register contents to the DAC
parameter [15:0] Addr = 16'b0000000000000100;		//Send the 7-bit i2c address plus write bit
parameter [15:0] Ack1 = 16'b0000000000001000;		//Look for acknowledge from the DAC
parameter [15:0] Byt1 = 16'b0000000000010000;		//Send 8 bits to the DAC
parameter [15:0] Ack2 = 16'b0000000000100000;		//Look for acknowledge from the DAC
parameter [15:0] Byt2 = 16'b0000000001000000;		//Send another 8 bits to the DAC
parameter [15:0] Ack3 = 16'b0000000010000000;		//Look for acknowledge from the DAC
parameter [15:0] RdRg = 16'b0000000100000000;		//Read register contents from the DAC
parameter [15:0] Add1 = 16'b0000001000000000;		//Send the 7 bit i2c address plus read bit
parameter [15:0] Ack4 = 16'b0000010000000000;		//Look for acknowledge from the DAC
parameter [15:0] Byt3 = 16'b0000100000000000;		//Receive 8 bits from the DAC
parameter [15:0] Ack5 = 16'b0001000000000000;		//Send an acknowledge to the DAC
parameter [15:0] Byt4 = 16'b0010000000000000;		//Receive another 8 bits from the DAC
parameter [15:0] Nack = 16'b0100000000000000;		//Send a NACK to the DAC
parameter [15:0] Endt = 16'b1000000000000000;		//Set the stop signal to the DAC

reg [15:0] State, NextState;
reg [7:0] AddrI2C;
reg [15:0] RegI2C;
reg ReadEn;
reg SDAinLat;

always @ (State or CntBit or StrtSnd or StrtRead or RegI2C or SlowClk or AddrI2C or SDAinLat) begin
	case (State)
		Wait:	begin
					ReadEn = 1'b0;
					SDAout = 1'b1;
					if (StrtSnd) begin
						NextState = SndR;
						SDAen = 1'b1;
					end else if (StrtRead) begin
						NextState = RdRg;
						SDAen = 1'b1;
					end else begin
						NextState = Wait;
						SDAen = 1'b0;
					end
				end
		SndR:	begin
					ReadEn = 1'b0;
					SDAout = 1'b0;	
					SDAen = 1'b1;
					NextState = Addr;
				end
		Addr:	begin
					ReadEn = 1'b0;
					SDAen = 1'b1;
					SDAout = AddrI2C[7];
					if (CntBit == 7) NextState = Ack1;
					else NextState = Addr;
				end
		Ack1:	begin
					ReadEn = 1'b0;
					SDAen = 1'b0;
					SDAout = 1'b1;
					if (SDAinLat) NextState = Endt;		//Failure of DAC to acknowledge
					else NextState = Byt1;
				end
		Byt1:	begin
					ReadEn = 1'b0;
					SDAen = 1'b1;
					SDAout = RegI2C[15];
					if (CntBit == 7) NextState = Ack2;
					else NextState = Byt1;
				end
		Ack2:	begin
					ReadEn = 1'b0;
					SDAen = 1'b0;
					SDAout = 1'b1;
					if (SDAinLat) NextState = Endt;
					else NextState = Byt2;
				end
		Byt2:	begin
					ReadEn = 1'b0;
					SDAen = 1'b1;
					SDAout = RegI2C[15];
					if (CntBit == 7) NextState = Ack3;
					else NextState = Byt2;
				end
		Ack3:	begin
					ReadEn = 1'b0;
					SDAen = 1'b0;
					SDAout = 1'b1;
					NextState = Endt;
				end
		RdRg:	begin		
					ReadEn = 1'b0;		
					SDAen = 1'b1;
					SDAout = 1'b0;
					NextState = Add1;
				end
		Add1:	begin
					ReadEn = 1'b0;
					SDAen = 1'b1;
					SDAout = AddrI2C[7];
					if (CntBit == 7) NextState = Ack4;
					else NextState = Add1;
				end
		Ack4:	begin
					ReadEn = 1'b0;
					SDAen = 1'b0;
					SDAout = 1'b1;
					if (SDAinLat) NextState = Endt;		//Failure of DAC to acknowledge
					else NextState = Byt3;
				end
		Byt3:	begin
					ReadEn = 1'b1;
					SDAen = 1'b0;
					SDAout = 1'b1;
					if (CntBit == 7) NextState = Ack5;
					else NextState = Byt3;	
				end
		Ack5:	begin
					ReadEn = 1'b0;
					SDAen = 1'b1;
					SDAout = 1'b0;
					NextState = Byt4;
				end
		Byt4:	begin
					SDAen = 1'b0;
					SDAout = 1'b1;
					ReadEn = 1'b1;
					if (CntBit == 7) begin
						NextState = Nack;
					end else begin
						NextState = Byt4;
					end
				end
		Nack:	begin
					ReadEn = 1'b0;
					SDAen = 1'b1;
					SDAout = 1'b1;
					NextState = Endt;
				end
		Endt:	begin
					ReadEn = 1'b0;
					SDAen = 1'b1;
					NextState = Wait;
					SDAout = 1'b0;
				end
		default: begin
					ReadEn = 1'b0;
					SDAen = 1'b0;
					SDAout = 1'b1;
					NextState = Wait;
				 end
	endcase
end

reg SDAprt, SCLprt;
always @ (SDAout or SDAen) begin
	if (SDAen) SDAprt = SDAout;
	else SDAprt = 1'b1;
end
always @ (SCL or SCLen) begin
	if (SCLen) SCLprt = SCL;
	else SCLprt = 1'b1;
end
initial begin
//	$display("                Time       State      SlowClk SCL SCLen SDAin SDAout SDAen ReadEn DoneSnd DoneRead CntBit   AddrI2C    RegI2C    StrtSnd    RegOut");
//	$monitor($time,"  %b   %b      %b    %b    %b     %b      %b      %b      %b       %b       %d    %b %b %b %b",State,SlowClk,SCLprt,SCLen,SDAin,SDAprt,SDAen,ReadEn,DoneSnd,DoneRead,CntBit,AddrI2C,RegI2C,StrtSnd,RegOut);
end

always @ (posedge SCL) begin
		if (ReadEn) RegOut <= {RegOut[14:0],SDAin};  //Shift in the register data
end

//Use the rising edge of SlowClk2 to control the enable and disable of SCL,
//in order to get the correct i2c start and stop conditions.  Note that SCL
//and SDA cannot transition at the same time, hence the need for the phase
//difference between SlowClk and SlowClk2.
always @ (posedge SlowClk2) begin
	if (ResetI2C) begin
		SCLen <= 1'b0;
	end else begin
		SDAinLat <= SDAin;    //Grab and hold the acknowledge signal long enough for it to be viewed
		case (State)
			Wait:	begin
						SCLen <= 1'b0;
					end
			SndR:	begin
						SCLen <= 1'b1;
					end
			RdRg:	begin
						SCLen <= 1'b1;
					end
			Endt:	begin
						SCLen <= 1'b0;
					end
		endcase
	end
end

//The rising edge of SlowClk controls all of the SDA data transitions.
always @ (posedge SlowClk) begin
	if (ResetI2C) begin
		State <= Wait;
		DoneSnd <= 1'b0;
		DoneRead <= 1'b0;
		Error <= 0;
	end else begin
		State <= NextState;
		case (State) 
			Wait:	begin
						StrobeOut <= 1'b0;
					end
			SndR:	begin
						CntBit <= 0;
						AddrI2C <= {Address,1'b0};   //Address plus write bit
						RegI2C <= RegLatch;
						Error <= 0;
					end
			Addr:	begin
						CntBit <= CntBit + 1;
						AddrI2C <= {AddrI2C[6:0],1'b0};
					end
			Ack1:	begin
						CntBit <= 0;
						if (SDAinLat) begin
							DoneSnd <= 1'b1;
							Error[0] <= 1'b1;
						end
					end
			Byt1:	begin
						CntBit <= CntBit + 1;
						RegI2C <= {RegI2C[14:0],1'b0};
					end
			Ack2:	begin
						CntBit <= 0;
						if (SDAinLat) begin
							DoneSnd <= 1'b1;
							Error[1] <= 1'b1;
						end
					end
			Byt2:	begin
						CntBit <= CntBit + 1;
						RegI2C <= {RegI2C[14:0],1'b0};
					end
			Ack3:	begin
						DoneSnd <= 1'b1;
						if (SDAinLat) Error <= 2'b11;
					end
			RdRg:	begin
						CntBit <= 0;
						AddrI2C <= {Address,1'b1};   //Address plus read bit
						Error <= 0;
					end
			Add1:	begin
						CntBit <= CntBit + 1;
						AddrI2C <= {AddrI2C[6:0],1'b0};
					end
			Ack4:	begin
						CntBit <= 0;
						if (SDAinLat) begin
							DoneRead <= 1'b1;
							Error[0] <= 1'b1;
						end
					end
			Byt3:	begin
						CntBit <= CntBit + 1;
					end
			Ack5:	begin
						CntBit <= 1'b0;
					end
			Byt4:	begin
						CntBit <= CntBit + 1;
					end
			Nack:	begin
						DoneRead <= 1'b1;
					end
			Endt:	begin
						DoneSnd <= 1'b0;
						DoneRead <= 1'b0;
						if (DoneRead) StrobeOut <= 1'b1;
					end
		endcase
	end
end

endmodule
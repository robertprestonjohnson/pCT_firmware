// Calculate the trigger rate every 1/10 second, store it in RAM,
// and send it to the UART link at the end of the run.
// A 24-bit trigger count is followed by a 24-bit event count, and repeated. . .
// R. Johnson    November 3, 2013
// Modified 1/19/2014 to include counts of events written (trigger accepted) to monitor dead time
// Modified 2/4/2015 to report when the rate is above a threshold of 1 kHz, for detecting the end of spill
module BeamMonitor(SpillFlag, DmpDone, TxD_Start, TxD_Data, TxD_Busy, DmpStart, Trigger, EvtTrigger, RunInProg, Clock, Reset);
input Clock;
input Reset;
input RunInProg;
input Trigger;		//One-clock-long trigger signal
input EvtTrigger;	//One-clock-long signal when an event is taken
input DmpStart;		//Signal to start sending out the data at the end of the run
input TxD_Busy;		//True if the UART is busy and cannot accept data
output TxD_Start;	//Signal that a byte of data is ready for the UART
output [7:0] TxD_Data;	//Data byte output to the UART
output DmpDone;		//Signal that all the data have been sent out
output SpillFlag;   //True if the last measurement of the trigger rate was > 1 kHz, to detect end of spill

reg TxD_Start;
reg [7:0] TxD_Data;
reg SpillFlag;

//State machine to count the triggers and write the counts to memory
parameter [2:0] Cnts=3'b001;
parameter [2:0] Stor=3'b010;
parameter [2:0] Str2=3'b100;

reg [2:0] State, NextState;
reg [23:0] Counter, EvtCntr;
reg [23:0] Timer;
reg [10:0] writeAddress;
reg [23:0] dataWrite;
reg writeEnable;
reg DmpDone;

always @ (State or Timer or writeAddress or Counter or EvtCntr) begin
	case (State)
		Cnts:	begin
					if (Timer == 24'd10000000) NextState = Stor;   //Should be 10 million for 100 MHz clock
					else NextState = Cnts;
					writeEnable = 1'b0;
					dataWrite = Counter;
				end
		Stor:	begin
					NextState = Str2;
					writeEnable = 1'b1;
					dataWrite = Counter;
				end
		Str2:	begin
					NextState = Cnts;
					writeEnable = 1'b1;
					dataWrite = EvtCntr;
				end
		default: 	begin
						NextState = Cnts;
						writeEnable = 1'b0;
						dataWrite = Counter;
					end
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin
		State <= Cnts;
		Counter <= 0;
		EvtCntr <= 0;
		Timer <= 0;
		SpillFlag <= 1'b0;
		writeAddress <= 0;
	end else begin
		State <= NextState;
		case (State)
			Cnts:	begin
						if (DmpDone) writeAddress <= 0;
						if (RunInProg) begin
							Timer <= Timer + 1;
							if (Trigger) Counter <= Counter + 1;
							if (EvtTrigger) EvtCntr <= EvtCntr + 1;
						end
					end
			Stor:	begin
						if (RunInProg && EvtTrigger) EvtCntr <= EvtCntr + 1;
						Timer <= 0;
						Counter <= 0;
						if (writeAddress != 11'b11111111111) writeAddress <= writeAddress + 1;
						SpillFlag <= (Counter > 10);
						$display("%g\t BeamMonitor:  Storing trigger count %d in address %d,  time=%d, writeEnable=%b.",$time,Counter,writeAddress,Timer,writeEnable);
					end
			Str2:	begin
						EvtCntr <= 0;
						if (RunInProg) begin
							if (Trigger) Counter <= Counter + 1;
							Timer <= Timer + 1;
						end
						writeAddress <= writeAddress + 1;
						$display("%g\t BeamMonitor:  Storing event count %d in address %d,  time=%d, writeEnable=%b.",$time,EvtCntr,writeAddress,Timer,writeEnable);
					end
		endcase
	end
	
end

//State machine to read the counts from the memory and send them to the UART
parameter [12:0] WaitSd = 13'b0000000000001;	//Wait for the signal to send data out
parameter [12:0] SdL1Sd = 13'b0000000000010;	//Send the first byte of the buffer length
parameter [12:0] Slp1Sd = 13'b0000000000100;
parameter [12:0] SdL2Sd = 13'b0000000001000;	//Send the second byte of the buffer length
parameter [12:0] Slp2Sd = 13'b0000000010000;
parameter [12:0] SdD1Sd = 13'b0000000100000;	//Send the first byte of a data word
parameter [12:0] Slp3Sd = 13'b0000001000000;
parameter [12:0] SdD2Sd = 13'b0000010000000;	//Send the second byte of a data word
parameter [12:0] Slp4Sd = 13'b0000100000000;
parameter [12:0] SdD3Sd = 13'b0001000000000;	//Send the third byte of a data word
parameter [12:0] Slp5Sd = 13'b0010000000000;
parameter [12:0] NextSd = 13'b0100000000000;	//Get ready for the next data word
parameter [12:0] SdDnSd = 13'b1000000000000;

reg [12:0] StateSd, NextStateSd;
reg readEnable;
reg [10:0] readAddress;
parameter [2:0] Dly= 3'h2; //Set length of time to assert TxD_start
reg [2:0] CntWt;

always @ (StateSd or DmpStart or readAddress or writeAddress or dataOut or TxD_Busy or CntWt) begin
	case (StateSd)
		WaitSd:	begin
					if (DmpStart && writeAddress > 0) NextStateSd = SdL1Sd;
					else NextStateSd = WaitSd;
					TxD_Data = 0;
					readEnable = 1'b0;
				end
		SdL1Sd:	begin
					if (TxD_Busy) NextStateSd = SdL1Sd;
					else NextStateSd = Slp1Sd;
					TxD_Data = {6'b000000,writeAddress[9:8]};
					readEnable = 1'b0;
				end
		Slp1Sd:	begin
					if (CntWt==Dly) NextStateSd = SdL2Sd;
					else NextStateSd = Slp1Sd;
					TxD_Data = {6'b000000,writeAddress[9:8]};
					readEnable = 1'b0;
				end
		SdL2Sd:	begin
					if (TxD_Busy) NextStateSd = SdL2Sd;
					else NextStateSd = Slp2Sd;
					TxD_Data = {6'b000000,writeAddress[9:8]};
					readEnable = 1'b0;
				end
		Slp2Sd:	begin
					if (CntWt==Dly) NextStateSd = SdD1Sd;
					else NextStateSd = Slp2Sd;
					TxD_Data = writeAddress[7:0];
					readEnable = 1'b1;
				end
		SdD1Sd:	begin
					if (TxD_Busy) NextStateSd = SdD1Sd;
					else NextStateSd = Slp3Sd;
					readEnable = 1'b0;
					TxD_Data = writeAddress[7:0];
				end
		Slp3Sd:	begin
					if (CntWt==Dly) NextStateSd = SdD2Sd;
					else NextStateSd = Slp3Sd;
					readEnable = 1'b0;
					TxD_Data = dataOut[23:16];
				end
		SdD2Sd:	begin
					if (TxD_Busy) NextStateSd = SdD2Sd;
					else NextStateSd = Slp4Sd;
					TxD_Data = dataOut[23:16];
					readEnable = 1'b0;
				end
		Slp4Sd:	begin
					if (CntWt==Dly) NextStateSd = SdD3Sd;
					else NextStateSd = Slp4Sd;
					TxD_Data = dataOut[15:8];
					readEnable = 1'b0;
				end
		SdD3Sd:	begin
					if (TxD_Busy) NextStateSd = SdD3Sd;
					else NextStateSd = Slp5Sd;
					TxD_Data = dataOut[15:8];
					readEnable = 1'b0;
				end
		Slp5Sd:	begin
					if (CntWt==Dly) NextStateSd = NextSd;
					else NextStateSd = Slp5Sd;
					TxD_Data = dataOut[7:0];
					readEnable = 1'b0;
				end
		NextSd:	begin
					if (readAddress == writeAddress) NextStateSd = WaitSd;
					else NextStateSd = SdDnSd;
					readEnable = 1'b1;
					TxD_Data = dataOut[7:0];
				end
		SdDnSd:	begin
					if (TxD_Busy) NextStateSd = SdDnSd;
					else NextStateSd = Slp3Sd;
					TxD_Data = dataOut[23:16];
					readEnable = 1'b0;
				end
		default:	begin
						NextStateSd = WaitSd;
						readEnable = 1'b0;
						TxD_Data = 0;
					end
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin
		StateSd <= WaitSd;
	end else begin
		StateSd <= NextStateSd;
		if (StateSd != WaitSd) $display("%g\t BeamMonitor StateSd=%b, readAddress=%d, TxD_Start=%b, TxD_Data=%b, readEnable=%b, dataOut=%b, TxD_Busy=%b, DmpDone=%b.",$time,StateSd,readAddress,TxD_Start,TxD_Data,readEnable,dataOut,TxD_Busy,DmpDone);
		case (StateSd) 
			WaitSd:	begin
						TxD_Start <= 1'b0;
						readAddress <= 0;
						DmpDone <= 1'b0;
						CntWt <= 0;
						if (DmpStart) $display("%g\t BeamMonitor:  start dumping %d words.",$time,writeAddress);
					end
			Slp1Sd:	begin
						TxD_Start <= 1'b1;
						CntWt <= CntWt + 1;
					end
			SdL2Sd:	begin
						CntWt <= 0;
						TxD_Start <= 1'b0;
					end
			Slp2Sd:	begin
						CntWt <= CntWt + 1;
						TxD_Start <= 1'b1;
					end
			SdD1Sd:	begin
						TxD_Start <= 1'b0;
						CntWt <= 0;
					end
			Slp3Sd:	begin
						CntWt <= CntWt + 1;
						TxD_Start <= 1'b1;
					end
			SdD2Sd:	begin
						TxD_Start <= 1'b0;
						CntWt <= 0;
					end
			Slp4Sd:	begin
						TxD_Start <= 1'b1;
						CntWt <= CntWt + 1;
					end
			SdD3Sd:	begin
						if (!TxD_Busy) begin
							readAddress <= readAddress + 1;
						end
						TxD_Start <= 1'b0;
						CntWt <= 0;
					end
			Slp5Sd:	begin
						TxD_Start <= 1'b1;
						CntWt <= CntWt + 1;
					end
			NextSd:	begin
						TxD_Start <= 1'b0;
						if (NextStateSd == WaitSd) DmpDone <= 1'b1;
					end
			SdDnSd:	begin
						CntWt <= 0;
					end
		endcase
	end
end

wire [23:0] doa, dataOut;
BeamMonRAM BeamMonRAM0(Clock, Clock, writeEnable, readEnable, writeEnable, writeAddress, readAddress, dataWrite, doa, dataOut); 

endmodule

module BeamMonRAM (clka, clkb, ena, enb, wea, addra, addrb, dia, doa, dob);
input clka, clkb;
input wea;                  //Write enable
input ena, enb;             //Primary and secondary read enables
input [10:0] addra, addrb;   //Addresses for the primary and secondary ports
input [23:0] dia;           //Input data register to write to memory
output [23:0] doa, dob;     //Output registers for the two ports
reg [23:0] RAM [2047:0];    //Memory array
reg [23:0] dob, doa;

always @(posedge clka)
begin
	if (ena) begin
		if (wea) begin
			RAM[addra]<=dia;  
			doa <= dia;
			$display("%g\t BeamMonRAM storing %b in address %d",$time,dia,addra);
		end else doa <= RAM[addra];        //Write first, then read
	end
end

always @(posedge clkb)
begin
	if (enb) begin
		dob <= RAM[addrb];
		$display("%g\t BeamMonRAM: reading %b from address %d",$time,RAM[addrb],addrb);
	end
end
endmodule

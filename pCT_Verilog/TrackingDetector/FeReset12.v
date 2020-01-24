//Program to run in the pCT Tracker front-end board to reset the
//ASICs without affecting the data acquisition registers.  To do 
//that, it must read the present configuration and store it, send
//the reset command, and then reload the configuration.  A wait
//is needed after that to allow the DAC and I/O channels to 
//settle.  Only the configuration register, trigger and data mask
//registers, and the threshold DAC register need to be reloaded.
//R. Johnson, 2013
module FeReset(Done,Cmd,Data,Start,Clock,Reset);

input Reset;
input Clock;
input Start;		//Signal to start the reset
input [11:0] Data;  	//Data lines from the 12 chips

output Cmd;        	//Commands going out to the chips
output Done;		//Signal that all is completed and the run can resume

reg Cmd;
reg Done;

wire [2:0] Error0, Error1, Error2, Error3, Error4, Error5, Error6, Error7, Error8, Error9, ErrorA, ErrorB;
reg [11:0] Send;
wire [11:0] Cmd1;
wire [11:0] Done1;

initial begin
//	$display("Time           State     Start  Data   Cmd  Send   Cmd1  Done1   Command   Cnt   CntWt");
end
always @ (posedge Clock) begin
//	$display("%g\t  %b %b   %b   %b  %b %b %b %b %d %d",$time,State,Start,Data,Cmd,Send,Cmd1,Done1,Command,Cnt,CntWt);
end

//Instantiate a program to handle each of the ASICs
//They will all store the register information simultaneously,
//and they will be reset simultaneously but they will be reloaded 
//one at a time (there is only a single command line).
Fe1Rst Fe1Rst0(Done1[0],Error0,Cmd1[0],Data[0],Send[0],Start,5'b00000,Clock,Reset);
Fe1Rst Fe1Rst1(Done1[1],Error1,Cmd1[1],Data[1],Send[1],Start,5'b00001,Clock,Reset);
Fe1Rst Fe1Rst2(Done1[2],Error2,Cmd1[2],Data[2],Send[2],Start,5'b00010,Clock,Reset);
Fe1Rst Fe1Rst3(Done1[3],Error3,Cmd1[3],Data[3],Send[3],Start,5'b00011,Clock,Reset);
Fe1Rst Fe1Rst4(Done1[4],Error4,Cmd1[4],Data[4],Send[4],Start,5'b00100,Clock,Reset);
Fe1Rst Fe1Rst5(Done1[5],Error5,Cmd1[5],Data[5],Send[5],Start,5'b00101,Clock,Reset);
Fe1Rst Fe1Rst6(Done1[6],Error0,Cmd1[6],Data[6],Send[6],Start,5'b00110,Clock,Reset);
Fe1Rst Fe1Rst7(Done1[7],Error1,Cmd1[7],Data[7],Send[7],Start,5'b00111,Clock,Reset);
Fe1Rst Fe1Rst8(Done1[8],Error2,Cmd1[8],Data[8],Send[8],Start,5'b01000,Clock,Reset);
Fe1Rst Fe1Rst9(Done1[9],Error3,Cmd1[9],Data[9],Send[9],Start,5'b01001,Clock,Reset);
Fe1Rst Fe1RstA(Done1[10],Error4,Cmd1[10],Data[10],Send[10],Start,5'b01010,Clock,Reset);
Fe1Rst Fe1RstB(Done1[11],Error5,Cmd1[11],Data[11],Send[11],Start,5'b01011,Clock,Reset);

//State definitions for the state machine
parameter [15:0] Wait=16'b0000000000000001;	//Wait for the start signal
parameter [15:0] CmTh=16'b0000000000000010;	//Load the read threshold DAC register command
parameter [15:0] RdTh=16'b0000000000000100;	//Send out the read threshold DAC register command
parameter [15:0] CmCf=16'b0000000000001000;	//Load the read configuration register command
parameter [15:0] RdCf=16'b0000000000010000;	//Send out the read configuration register command
parameter [15:0] CmDt=16'b0000000000100000;	//Load the read data mask register command
parameter [15:0] RdDt=16'b0000000001000000;	//Send out the read data mask register command
parameter [15:0] CmTg=16'b0000000010000000;	//Load the read trigger mask register command
parameter [15:0] RdTg=16'b0000000100000000;	//Send out the read trigger mask register command
parameter [15:0] CmRs=16'b0000001000000000;	//Load the reset command
parameter [15:0] SdRs=16'b0000010000000000;	//Send out the reset command
parameter [15:0] LdCh=16'b0000100000000000;	//Send a signal to a chip to load its registers
parameter [15:0] WtCh=16'b0001000000000000;	//Wait for a chip to finish loading its registers
parameter [15:0] NxCh=16'b0010000000000000;	//Increment the chip pointer
parameter [15:0] Fini=16'b0100000000000000;	//Wait for the electronics to settle
parameter [15:0] SdDn=16'b1000000000000000;	//Send the done signal

reg [15:0] State, NextState;
reg [10:0] Command;
reg [3:0] Cnt;
reg [15:0] CntWt;
reg [3:0] Idx;			//Pointer to the chip being loaded

always @ (State or Start or Cnt or CntWt or Done1 or Command or Cmd1 or Idx) begin
	case (State)
		Wait:	begin
					Cmd = 1'b0;
					if (Start) NextState = CmTh;
					else NextState = Wait;
				end
		CmTh:	begin
					Cmd = 1'b0;
					NextState = RdTh;
				end
		RdTh:	begin
					if (Cnt == 10) NextState = CmCf;
					else NextState = RdTh;
					Cmd = Command[10];
				end
		CmCf:	begin
					Cmd = 1'b0;
					if (CntWt == 75) NextState = RdCf;
					else NextState = CmCf;
				end
		RdCf:	begin
					if (Cnt == 10) NextState = CmDt;
					else NextState = RdCf;
					Cmd = Command[10];
				end
		CmDt:	begin
					Cmd = 1'b0;
					if (CntWt == 75) NextState = RdDt;
					else NextState = CmDt;
				end
		RdDt:	begin
					if (Cnt == 10) NextState = CmTg;
					else NextState = RdDt;
					Cmd = Command[10];
				end
		CmTg:	begin
					Cmd = 1'b0;
					if (CntWt == 75) NextState = RdTg;
					else NextState = CmTg;
				end
		RdTg:	begin
					if (Cnt == 10) NextState = CmRs;
					else NextState = RdTg;
					Cmd = Command[10];
				end
		CmRs:	begin
					Cmd = 1'b0;
					if (CntWt == 75) NextState = SdRs;
					else NextState = CmRs;
				end
		SdRs:	begin
					if (Cnt == 10) NextState = LdCh;
					else NextState = SdRs;
					Cmd = Command[10];
				end
		LdCh:	begin
					Cmd = Cmd1[Idx];
					NextState = WtCh;
				end
		WtCh:	begin
					Cmd = Cmd1[Idx];
					if (Done1[Idx]) begin
						if (Idx == 4'b1011) NextState = Fini;
						else NextState = NxCh;
					end else NextState = WtCh;
				end
		NxCh:	begin
					Cmd = 1'b0;
					NextState = LdCh;
				end
		Fini:	begin
					Cmd = 1'b0;
					if (CntWt == 16'b1111111111111111) NextState = SdDn;
//					if (CntWt == 16'b0000000000111111) NextState = SdDn;
					else NextState = Fini;
				end
		SdDn:	begin
					Cmd = 1'b0;
					NextState = Wait;
				end
		default:	begin
						NextState = Wait;
						Cmd = 1'b0;
					end
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin
		State <= Wait;
		Send <= 0;
	end else begin
		State <= NextState;
		case (State)
			Wait:	begin
						Done <= 1'b0;
					end
			CmTh:	begin
						Command <= 11'b11111101000;
						Cnt <= 0;
						Idx <= 0;
						CntWt <= 0;
					end
			RdTh:	begin
						Cnt <= Cnt + 1;
						Command <= {Command[9:0],1'b0};
					end
			CmCf:	begin
						Cnt <= 0;
						CntWt <= CntWt + 1;
						Command <= 11'b11111101011;
					end
			RdCf:	begin
						CntWt <= 0;
						Cnt <= Cnt + 1;
						Command <= {Command[9:0],1'b0};
					end
			CmDt:	begin
						Cnt <= 0;
						CntWt <= CntWt + 1;
						Command <= 11'b11111101101;
					end
			RdDt:	begin
						CntWt <= 0;
						Cnt <= Cnt + 1;
						Command <= {Command[9:0],1'b0};
					end
			CmTg:	begin
						Cnt <= 0;
						CntWt <= CntWt + 1;
						Command <= 11'b11111101110;
					end
			RdTg:	begin
						Cnt <= Cnt + 1;
						CntWt <= 0;
						Command <= {Command[9:0],1'b0};
					end
			CmRs:	begin
						Cnt <= 0;
						CntWt <= CntWt + 1;
						Command <= 11'b11111100010;
					end
			SdRs:	begin
						Cnt <= Cnt + 1;
						Command <= {Command[9:0],1'b0};
					end
			LdCh:	begin
						Send[Idx] <= 1'b1;
						CntWt <= 0;
					end
			WtCh:	begin
						Send[Idx] <= 1'b0;
					end
			NxCh:	begin
						Idx <= Idx + 1;
					end
			Fini:	begin
						CntWt <= CntWt + 1;
					end
			SdDn:	begin
						Done <= 1'b1;
					end
		endcase
	end
end

endmodule

//This module handles a single ASIC.  It stores the register information
//as it flows in and calculates the parity.  When signaled, for each
//register it forms the appropriate load register command and shifts
//it out on the Cmd line.
module Fe1Rst(Done,Error,Cmd,Data,Send,Start,Address,Clock,Reset);

input Reset;
input Clock;
input [4:0] Address;	//Address of the chip this instance will handle
input Start;			//Signal to start looking for register data
input Send;				//Signal to start sending load register commands
input Data;				//Data from a single chip

output Cmd;				//Command stream to the chips
output [2:0] Error;		//Error bits from the configuration register
output Done;			//Signal that loading is done

reg Cmd, Done;
wire [2:0] Error;

reg [5:0] Header;
reg [63:0] TrgMsk, DatMsk;
reg [21:0] Config;
reg [7:0] ThrDac;
reg [2:0] Cnt;
reg [6:0] CntBits;
reg [74:0] Command;
reg PrtyThr, PrtyConf, PrtyDat, PrtyTrg;

assign PrtyAddr = Address[0]^Address[1]^Address[2]^Address[3]^Address[4];
assign Error = Config[21:19];

parameter [15:0] Wait=16'b0000000000000001;	//Wait for a start signal
parameter [15:0] Redy=16'b0000000000000010;  //Look for the begining of a data stream
parameter [15:0] Hedr=16'b0000000000000100;	//Shift in the 6-bit header
parameter [15:0] Thrs=16'b0000000000001000;	//Store threshold DAC bits
parameter [15:0] Conf=16'b0000000000010000;	//Store configuration register bits
parameter [15:0] DatM=16'b0000000000100000;	//Store data mask bits
parameter [15:0] TrgM=16'b0000000001000000;	//Store trigger mask bits
parameter [15:0] CmTh=16'b0000000010000000;	//Set up command for DAC register
parameter [15:0] LdTh=16'b0000000100000000;	//Load threshold DAC register
parameter [15:0] CmCf=16'b0000001000000000;	//Set up command for configuration register
parameter [15:0] LdCf=16'b0000010000000000;	//Load configuration register
parameter [15:0] CmDt=16'b0000100000000000;	//Set up command for data mask register
parameter [15:0] LdDt=16'b0001000000000000;	//Load data mask register
parameter [15:0] CmTg=16'b0010000000000000;	//Set up command for trigger mask register
parameter [15:0] LdTg=16'b0100000000000000;	//Load trigger mask register
parameter [15:0] SdDn=16'b1000000000000000;	//Send done signal

reg [15:0] State, NextState;

always @ (State or Start or Data or Send or Header or Cnt or CntBits or Command) begin
	case (State)
		Wait:	begin
					Cmd = 1'b0;
					if (Start) NextState = Redy;
					else NextState = Wait;
				end
		Redy:	begin
					Cmd = 1'b0;
					if (Send) NextState = CmTh;
					else if (Data) NextState = Hedr;
					else NextState = Redy;
				end
		Hedr:	begin
					Cmd = 1'b0;
					if (Cnt == 4) begin
						case (Header[2:0])
							3'b010:	NextState = Thrs;
							3'b011: NextState = Conf;
							3'b100: NextState = DatM;
							3'b101: NextState = TrgM;
							default: NextState = SdDn;
						endcase
					end else begin
						NextState = Hedr;
					end
				end
		Thrs:	begin
					Cmd = 1'b0;
					if (CntBits == 65) NextState = Redy;
					else NextState = Thrs;
				end
		Conf:	begin
					Cmd = 1'b0;
					if (CntBits == 65) NextState = Redy;
					else NextState = Conf;
				end
		DatM:	begin
					Cmd = 1'b0;
					if (CntBits == 65) NextState = Redy;
					else NextState = DatM;
				end
		TrgM:	begin
					Cmd = 1'b0;
					if (CntBits == 65) NextState = Redy;
					else NextState = TrgM;
				end
		CmTh:	begin
					Cmd = 1'b0;
					NextState = LdTh;
				end
		LdTh:	begin
					Cmd = Command[74];
					if (CntBits == 18) NextState = CmCf;
					else NextState = LdTh;
				end
		CmCf:	begin
					Cmd = 1'b0;
					NextState = LdCf;
				end
		LdCf:	begin
					Cmd = Command[74];
					if (CntBits == 29) NextState = CmDt;
					else NextState = LdCf;
				end
		CmDt:	begin
					Cmd = 1'b0;
					NextState = LdDt;
				end
		LdDt:	begin
					Cmd = Command[74];
					if (CntBits == 74) NextState = CmTg;
					else NextState = LdDt;
				end
		CmTg:	begin
					Cmd = 1'b0;
					NextState = LdTg;
				end
		LdTg:	begin
					Cmd = Command[74];
					if (CntBits == 74) NextState = SdDn;
					else NextState = LdTg;
				end
		SdDn:	begin
					Cmd = 1'b0;
					NextState = Wait;
				end
		default:	begin
						Cmd = 1'b0;
						NextState = Wait;
					end
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin
		State <= Wait;
	end else begin
		State <= NextState;
		case (State)
			Wait:	begin
						Done <= 1'b0;
						PrtyThr <= 1'b0;       //Parity of the 4-bit command string
						PrtyConf <= 1'b1;
						PrtyDat <= 1'b0; 
						PrtyTrg <= 1'b1;
					end
			Redy:	begin
						Header <= {Header[4:0],Data};
						Cnt <= 0;
						CntBits <= 0;
					end
			Hedr:	begin
						Header <= {Header[4:0],Data};
						Cnt <= Cnt + 1;
					end
			Thrs:	begin
						if (CntBits < 8) begin
							ThrDac <= {ThrDac[6:0],Data};
							PrtyThr <= PrtyThr^Data;    //Include the parity of the data field
//							$display("%g\t %h PrtyThr=%b,  Data=%b, Cnt=%d",$time,Address,PrtyThr,Data,CntBits);
						end
						CntBits <= CntBits + 1;
					end
			Conf:	begin
						if (CntBits < 22) begin
							Config <= {Config[20:0],Data};
							if (CntBits > 2) begin
								PrtyConf <= PrtyConf^Data;
//								$display("%g\t %h PrtyConf=%b,  Data=%b, Cnt=%d",$time,Address,PrtyConf,Data,CntBits);
							end
						end
						CntBits <= CntBits + 1;
					end
			DatM:	begin
						if (CntBits < 64) begin
							DatMsk <= {DatMsk[62:0],Data};
							PrtyDat <= PrtyDat^Data;
						end
						CntBits <= CntBits + 1;
					end
			TrgM:	begin
						if (CntBits < 64) begin
							TrgMsk <= {TrgMsk[62:0],Data};
							PrtyTrg <= PrtyTrg^Data;
						end
						CntBits <= CntBits + 1;
					end
			CmTh:	begin
						CntBits <= 0;
						Command <= {1'b1,Address,4'b1010,PrtyAddr^PrtyThr,ThrDac,56'b0};
					end
			LdTh:	begin
						CntBits <= CntBits + 1;
						Command <= {Command[73:0],1'b0};
					end
			CmCf:	begin
						CntBits <= 0;
//						$display("%g\t %h PrtyAddr=%b, PrtyConf=%b, Config=%b",$time,Address,PrtyAddr,PrtyConf,Config);
						Command <= {1'b1,Address,4'b1011,PrtyAddr^PrtyConf,Config[18:0],45'b0};
					end
			LdCf:	begin
//						if (CntBits == 0) $display("%g\t %h  Load config command = %b",$time,Address,Command);
						CntBits <= CntBits + 1;
						Command <= {Command[73:0],1'b0};
					end
			CmDt:	begin
						CntBits <= 0;
						Command <= {1'b1,Address,4'b1100,PrtyAddr^PrtyDat,DatMsk};
					end
			LdDt:	begin
						CntBits <= CntBits + 1;
						Command <= {Command[73:0],1'b0};
					end
			CmTg:	begin
						CntBits <= 0;
						Command <= {1'b1,Address,4'b1101,PrtyAddr^PrtyTrg,TrgMsk};
					end
			LdTg:	begin
						CntBits <= CntBits + 1;
						Command <= {Command[73:0],1'b0};
					end
			SdDn:	begin
						Done <= 1'b1;
					end
		endcase
	end
end

endmodule


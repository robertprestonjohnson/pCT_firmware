//Changes the data output from 12-bit wide words to 16-bit wide words, to go to
//the Ethernet output interface.
//R. Johnson  July 24, 2013
//July 28, 2013:  flush last word out at end of run
module EtherBuffer(DataOut,StrobeOut,DataIn,StrobeIn,endRun,Clock,Reset);
input Clock;
input Reset;
input endRun;			//1 clock signal that the run is ending
input StrobeIn;			//High each time a valid data word is present at the input
input [11:0] DataIn;	//12-bit input data word
output StrobeOut;		//High each time a valid data word is present at the output
output [15:0] DataOut;	//16-bit output data word

reg [15:0] DataOut;
reg StrobeOut;
reg [11:0] DataSave0;
reg [7:0] DataSave1;
reg [3:0] DataSave2;

//Algorithm is to output 3 words for every 4 that come in:
//read in word 1, no output (12 bits saved)
//read in word 2, output word 1 (8 bits saved)
//read in word 3, output word 2 (4 bits saved)
//read in word 4, output word 3 (0 bits saved)

parameter [3:0] Wrd1=4'b0001;
parameter [3:0] Wrd2=4'b0010;
parameter [3:0] Wrd3=4'b0100;
parameter [3:0] Wrd4=4'b1000;

reg [3:0] State, NextState;

always @ (State or StrobeIn) begin
	case (State)
		Wrd1:	begin
					if (StrobeIn) NextState=Wrd2;
					else NextState=Wrd1;
				end
		Wrd2:	begin
					if (StrobeIn) NextState=Wrd3;
					else NextState=Wrd2;
				end
		Wrd3:	begin
					if (StrobeIn) NextState=Wrd4;
					else NextState=Wrd3;
				end
		Wrd4:	begin
					if (StrobeIn) NextState=Wrd1;
					else NextState=Wrd4;
				end
		default: 	begin
						NextState = Wrd1;
					end
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin
		State <= Wrd1;
		StrobeOut <= 1'b0;
	end else begin
		State <= NextState;
		case (State)
			Wrd1:	begin
						if (StrobeIn) DataSave0 <= DataIn;
						StrobeOut <= 1'b0;
					end
			Wrd2:	begin
						if (StrobeIn) begin
							StrobeOut <= 1'b1;
							DataOut <= {DataSave0,DataIn[11:8]};
							DataSave1 <= DataIn[7:0];
						end else begin
							if (endRun) begin
								StrobeOut <= 1'b1;
								DataOut <= {DataSave0,4'h0};
							end else StrobeOut <= 1'b0;
						end
					end
			Wrd3:	begin
						if (StrobeIn) begin
							StrobeOut <= 1'b1;
							DataOut <= {DataSave1,DataIn[11:4]};
							DataSave2 <= DataIn[3:0];
						end else begin
							if (endRun) begin
								StrobeOut <= 1'b1;
								DataOut <= {DataSave1,8'h00};
							end else StrobeOut <= 1'b0;
						end
					end
			Wrd4:	begin
						if (StrobeIn) begin
							StrobeOut <= 1'b1;
							DataOut <= {DataSave2,DataIn};
						end else begin
							if (endRun) begin
								StrobeOut <= 1'b1;
								DataOut <= {DataSave2,12'h000};
							end else StrobeOut <= 1'b0;
						end
					end
		endcase
	end
end

endmodule

//Make the trigger decision based on energy detector and tracker input, as well as configuration settings.
//R. Johnson  November 1, 2013
//R. Johnson  March 31, 2014, modified to reduce delays
//R. Johnson  April 16, 2014, modified to return to the Wait state after a random trigger 
module TriggerLogic(NenrgMiss,NcosmicMiss,TrgPls,EnrgCoincidence,CosmicCoincidence,TrgStrchEx,TrgRndm,RunInProg,HoldEndR,TrgCnfg,CalTrgMsk,TkrTrgMsk,Clock,Reset);
input Clock;
input Reset;
input EnrgCoincidence;		//Energy detector trigger pulse
input CosmicCoincidence;	//Tracker trigger pulse
input TrgStrchEx;			//External trigger
input TrgRndm;				//Random trigger (for occupancy scans, for example)
input RunInProg;			//True if a run is in progress
input HoldEndR;				//Hold trigger at the end of a run to let all events flush out
input [3:0] TrgCnfg;		//Trigger configuration
input [1:0] CalTrgMsk;		//Energy detector trigger mask
input [7:0] TkrTrgMsk;		//Tracker trigger mask
output TrgPls;				//One clock-long pulse indicating a successful trigger
output [15:0] NenrgMiss, NcosmicMiss;  //For monitoring efficiency

reg TrgPls;

//Make a coincidence between the energy and tracker triggers, such that the timing edge is from the energy detector
//This will have to be much more sophisticated for our goal trigger rate, to pipeline process multiple triggers, or
//else simply use a calorimeter trigger
parameter [4:0] TR01=5'b00001;  //Look for the start of the tracker trigger signal
parameter [4:0] TR05=5'b00010;  //Look for the start of the energy trigger signal
parameter [4:0] TR06=5'b00100;  
parameter [4:0] TR07=5'b01000; 
parameter [4:0] TR04=5'b10000;  //Wait for trigger signals to go down
reg [4:0] StateTR, NextStateTR;
reg [5:0] CtrTR;

//For the combined tkr/enrg trigger, the tracker trigger should be zero delay but stretched long.  The
//enrg trigger should then be delayed to start within the tracker trigger window.  The coincidence 
//condition then is that the tracker trigger is high and then the energy trigger goes from low to high.

always @ (StateTR or TrgRndm or CtrTR or RunInProg or HoldEndR or TrgStrchEx or TrgCnfg or CalTrgMsk or EnrgCoincidence or CosmicCoincidence or TkrTrgMsk) begin
	case (StateTR)
		TR01: begin
					if (RunInProg & !HoldEndR) begin
						if (TrgCnfg[1]) begin  //External trigger
							if (TrgStrchEx) NextStateTR = TR04;
							else NextStateTR = TR01;
						end else if (TrgCnfg[0]) begin
							if (CalTrgMsk==2'b00) begin   //Tracker only trigger
								if (CosmicCoincidence) NextStateTR = TR04;
								else NextStateTR = TR01;
							end else if (TkrTrgMsk==8'h00) begin   //Energy only trigger
								if (EnrgCoincidence) begin
								    NextStateTR = TR04;
									$display("%g\t TriggerLogic: energy detector trigger.",$time);
								end	else NextStateTR = TR01;
							end else begin    //Combined tkr/enrg trigger
								if (CosmicCoincidence) NextStateTR = TR07;
								else if (EnrgCoincidence) NextStateTR = TR06;
								else NextStateTR = TR01;
							end
						end else begin    //Random trigger (occupancy scans)
							if (TrgRndm) NextStateTR = TR04;
							else NextStateTR = TR01;
						end
					end else NextStateTR = TR01;
				end
		TR06:	begin    //This little detour is meant only for keeping correct count of energy trigger pulses
					if (CosmicCoincidence) NextStateTR = TR07;
					else if (CtrTR > 5) NextStateTR = TR01;
					else NextStateTR = TR06;
				end
		TR07:	begin   //Here we have a tracker trigger, but wait for the energy trigger to be low before looking for the rising edge
					if (!CosmicCoincidence) NextStateTR = TR01;   
					else begin
						if (!EnrgCoincidence) NextStateTR = TR05;
						else NextStateTR = TR07;
					end
				end
		TR05:	begin   //In the combined tkr/enrg trigger, look for the rising edge of the energy trigger.  
					if (!CosmicCoincidence) NextStateTR = TR01;   
					else begin
						if (EnrgCoincidence) NextStateTR = TR04;   //Here we have a combined coincidence
						else NextStateTR = TR05;
					end
				end
		TR04:	begin  //Make sure the trigger pulses are low before looking for the next trigger
					if (TrgCnfg[1]) begin
						if (!TrgStrchEx) NextStateTR = TR01;
						else NextStateTR = TR04;
					end else if (TrgCnfg[0]) begin
						if (TkrTrgMsk==8'h00) begin   //Energy only trigger
							if (!EnrgCoincidence) NextStateTR = TR01;
							else NextStateTR = TR04;
						end else begin
							if (!CosmicCoincidence) NextStateTR = TR01;
							else NextStateTR = TR04;
						end
					end else begin
						NextStateTR = TR01;
					end
				end
		default: NextStateTR = TR01;
	endcase
end

reg [31:0] NTrgEx, NTrgEnrg, NTrgTkr, NTrgGen;
reg [15:0] NenrgMiss, NcosmicMiss;
reg cosmicLst, enrgLst;
always @ (posedge Clock) begin
	if (Reset) begin
		StateTR <= TR01;
		TrgPls <= 1'b0;
		NcosmicMiss <= 0;
		NenrgMiss <= 0;
	end else begin
		StateTR <= NextStateTR;
		cosmicLst <= CosmicCoincidence;
		enrgLst <= EnrgCoincidence;
		if (StateTR != TR01) begin
			if (CosmicCoincidence & !cosmicLst) NcosmicMiss <= NcosmicMiss + 1;
			if (EnrgCoincidence & !enrgLst) NenrgMiss <= NenrgMiss + 1;
		end
		if (NextStateTR==TR04) TrgPls <= 1'b1;
		case (StateTR)
			TR01: 	begin
						CtrTR <= 0;
					end
			TR06:	begin
						CtrTR <= CtrTR + 1;
					end
			TR04:	begin
						$display("%g\t TriggerLogic:  issuing a trigger pulse",$time);
						TrgPls <= 1'b0;
					end
		endcase
	end
end

endmodule

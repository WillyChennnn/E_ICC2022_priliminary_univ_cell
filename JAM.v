module JAM (
input CLK,
input RST,
output reg [2:0] W,
output reg [2:0] J,
input [6:0] Cost,
output reg [3:0] MatchCount,
output reg [9:0] MinCost,
output Valid );


parameter S_IDLE = 3'd0,
          S_CACULATE = 3'd1,
          S_FIND_RP = 3'd2,
          S_FIND_MIN = 3'd3,
          S_CHANGE_RP = 3'd4,
          S_DSEQ = 3'd5,
          S_FINISH = 3'd6;


wire find_rp, find_min;
wire [9:0] total;
wire [2:0] index;
wire [10:0] diff;
reg [2:0] state, n_state;
reg [15:0] seq_cnt;
reg [2:0] w_seq [0:7];
reg [2:0] caculate_cnt;
reg [2:0] rp;
reg [2:0] min;
reg [2:0] min_pointer;
reg [9:0] sum;
reg [2:0] pointer;
reg [2:0] min_record;

assign index = caculate_cnt;
assign diff = total - MinCost;
assign Valid = (state == S_FINISH)? 1'b1 : 1'b0;
assign find_rp = (~&rp)?1'b1:1'b0;
assign find_min = (min_pointer==rp)?1'b1:1'b0;
assign total = (&caculate_cnt)? sum+Cost : sum;

always@(posedge CLK or posedge RST)begin
    if(RST)begin
        state <= S_IDLE;
    end
    else begin
        state <= n_state;
    end
end
always@(*)begin
    case(state)
        S_IDLE:n_state = S_CACULATE;
        S_CACULATE:begin
            if(seq_cnt == 16'd40319 && &caculate_cnt)begin
                n_state = S_FINISH;
            end
            else if(&caculate_cnt)begin
                n_state = S_FIND_RP;
            end
            else begin
                n_state = state;
            end
        end
        S_FIND_RP:begin
            if(find_rp)begin
                n_state = S_FIND_MIN;
            end
            else begin
                n_state = state;
            end
        end
        S_FIND_MIN:begin
            if(find_min)begin
                n_state = S_DSEQ;
            end
            else begin
                n_state = state;
            end
        end
        S_CHANGE_RP: n_state = S_DSEQ;
        S_DSEQ:n_state = S_CACULATE;
	  S_FINISH: n_state = S_IDLE;
        default:n_state = S_IDLE;
    endcase
end

always@(posedge CLK or posedge RST)begin
    if(RST)begin
        w_seq[0] <= 3'd0;
        w_seq[1] <= 3'd1;
        w_seq[2] <= 3'd2;
        w_seq[3] <= 3'd3;
        w_seq[4] <= 3'd4;
        w_seq[5] <= 3'd5;
        w_seq[6] <= 3'd6;
        w_seq[7] <= 3'd7;
    end
    // change rp
    else if(state == S_FIND_MIN && min_pointer == rp)begin
        if(w_seq[min_record]>w_seq[rp])begin
            w_seq[min_record] <= w_seq[rp];
            w_seq[rp] <= w_seq[min_record];
        end
        else begin
        end
    end
    else if(state == S_DSEQ)begin
        case(rp)
            3'd0:begin
                w_seq[1] <= w_seq[7];
                w_seq[2] <= w_seq[6];
                w_seq[3] <= w_seq[5];
                w_seq[4] <= w_seq[4];
                w_seq[5] <= w_seq[3];
                w_seq[6] <= w_seq[2];
                w_seq[7] <= w_seq[1]; 
            end
            3'd1:begin
                w_seq[2] <= w_seq[7];
                w_seq[3] <= w_seq[6];
                w_seq[4] <= w_seq[5];
                w_seq[5] <= w_seq[4];
                w_seq[6] <= w_seq[3];
                w_seq[7] <= w_seq[2]; 
            end
            3'd2:begin
                w_seq[3] <= w_seq[7];
                w_seq[4] <= w_seq[6];
                w_seq[5] <= w_seq[5];
                w_seq[6] <= w_seq[4];
                w_seq[7] <= w_seq[3]; 
            end
            3'd3:begin
                w_seq[4] <= w_seq[7];
                w_seq[5] <= w_seq[6];
                w_seq[6] <= w_seq[5];
                w_seq[7] <= w_seq[4]; 
            end
            3'd4:begin
                w_seq[5] <= w_seq[7];
                w_seq[6] <= w_seq[6];
                w_seq[7] <= w_seq[5]; 
            end
            3'd5:begin
                w_seq[6] <= w_seq[7];
                w_seq[7] <= w_seq[6]; 
            end
        endcase
    end
end

// caculate cnt
always@(posedge CLK or posedge RST)begin
    if(RST)begin
        caculate_cnt <= 3'd0;
    end
    else if(state == S_CACULATE)begin
	  if(&caculate_cnt)begin
		caculate_cnt <= 3'd0;
	  end
	  else begin
		caculate_cnt <= caculate_cnt + 3'd1;
	  end
    end
    else begin
        caculate_cnt <= 3'd0;
    end
end

// W, J
always@(*)begin
    if(state == S_CACULATE)begin
	W = index;
	J = w_seq[index];
    end
    else begin
      W = 3'd0;
      J = 3'd0;
    end
end

always@(posedge CLK or posedge RST)begin
    if(RST)begin
        sum <= 10'd0;
    end
    else if(state == S_CACULATE)begin
        if(&caculate_cnt)begin
            sum <= 10'd0;
        end
        else begin
            sum <= sum + Cost;
        end
    end
    else begin
        sum <= 10'd0;
    end
end
always@(posedge CLK or posedge RST)begin
    if(RST)begin
        MinCost <= 10'd1023;
        MatchCount <= 4'd0;
    end
    else if(&caculate_cnt)begin
        if(diff[10])begin
            MinCost <= total;
            MatchCount <= 4'd1;
        end
	  else if(~|diff)begin
	      MinCost <= MinCost;
	      MatchCount <= MatchCount + 4'd1;
	  end
        else begin
            MinCost <= MinCost;
            MatchCount <= MatchCount;
        end
    end
    else begin
        MinCost <= MinCost;
        MatchCount <= MatchCount;
    end
end

// seq cnt
always@(posedge CLK or posedge RST)begin
    if(RST)begin
        seq_cnt <= 16'd0;
    end
    else if(state == S_DSEQ)begin
        seq_cnt <= seq_cnt + 16'd1;
    end
    else begin
    end
end

// compare

// find replace point
always@(posedge CLK or posedge RST)begin
    if(RST)begin
        pointer <= 3'd7;
        rp <= 3'd7;
    end
    else if(state == S_CACULATE)begin
	  rp <= 3'd7;
    end
    else if(state == S_FIND_RP)begin
        if(w_seq[pointer] > w_seq[pointer-3'd1])begin
            rp <= pointer - 3'd1;
            pointer <= 3'd7;
        end
        else begin
            rp <= rp;
            pointer <= pointer - 3'd1;
        end
    end
    else begin
        rp <= rp;
        pointer <= 3'd7;
    end
end

// find rp right-hand-side min
always@(posedge CLK or posedge RST)begin
    if(RST)begin
        min <= 3'd7;
        min_pointer <= 3'd7;
        min_record <= 3'd7;
    end
    else if(state == S_FIND_MIN)begin
        if(min_pointer > rp)begin
            min_pointer <= min_pointer -3'd1;
           if(w_seq[min_pointer] > w_seq[rp] && w_seq[min_pointer] <= min)begin
               min <= w_seq[min_pointer];
		   min_record <= min_pointer;
           end
           else begin
               min <= min;
           end
        end
        else begin
            min <= min;
        end
    end
    else begin
        min <= 3'd7;
	  min_pointer <= 3'd7;
    end
end


endmodule



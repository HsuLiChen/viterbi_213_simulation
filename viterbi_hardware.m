close all; clear; clc;

msg_source = [1 0 1 0 1 0]; 
msg_source = randi([0 1], 1, 8190);  % 這裡示範 50 bits
conv_code = conv_hardware_213(msg_source);

survivors = cell(4,1);       % 儲存各狀態的存活路徑
new_survivors = cell(4,1);  % 暫存新生成的存活路徑
path_metrics = [0;3;3;3];   % 各狀態的路徑度量值
decoded_msg = [];            % 儲存逐步輸出的解碼結果
D = 4;                       % 回溯深度設定
count = 0;
for step = 1:length(conv_code)/2
    idx = 2*step - 1;
    received_bits = conv_code(idx:idx+1);
    new_metrics = inf(4,1);  % 初始化新度量為無窮大
    new_survivors = cell(4,1); % 重置暫存器
    
    % 遍歷所有可能狀態
    for current_state = 0:3
        % 嘗試兩種可能的輸入位元
        for input_bit = 0:1
            % 取得次狀態和輸出
            next_state = viterbi_next_state(current_state, input_bit);
            output_dec = viterbi_outputs(current_state, input_bit);
            expected_bits = de2bi(output_dec, 2, 'left-msb');
            
            % 計算漢明距離
            hamming_dist = sum(received_bits ~= expected_bits);
            
            % 計算候選路徑度量
            candidate_metric = path_metrics(current_state+1) + hamming_dist;
            
            % 更新存活路徑（保留最優路徑）
            if candidate_metric < new_metrics(next_state+1)
                new_metrics(next_state+1) = candidate_metric;
                new_survivors{next_state+1} = [survivors{current_state+1}, input_bit];
            end
        end
    end
    
    % 更新全局變數
    path_metrics = new_metrics;
    survivors = new_survivors;
    
    % 執行回溯輸出（當有足夠歷史時）
    if step >= D
        [~, best_state] = min(path_metrics);
        best_path = survivors{best_state};
        decoded_bit = best_path(end-D+1);  % 取歷史中已確定的位元
        decoded_msg = [decoded_msg, decoded_bit];
        count = count +1;
        disp(count);
    end
end

% 處理剩餘位元（最後D-1個位元）
[~, final_state] = min(path_metrics);
remaining_bits = survivors{final_state}(end-D+2:end);
decoded_msg = [decoded_msg, remaining_bits];

disp('解碼結果:');
disp(decoded_msg);

% 驗證解碼正確性
isequal(msg_source, decoded_msg)
%---------VITERBI_HARDWARE_TABLE-----------
function nextState = viterbi_next_state(currentState,inptBits)
    switch currentState
        case 0 
            if(inptBits == 0)
                nextState = 0;
            else
                nextState = 2;
            end
         case 1 
            if(inptBits == 0)
                nextState = 0;
            else
                nextState = 2;
            end
         case 2 
            if(inptBits == 0)
                nextState = 1;
            else
                nextState = 3;
            end
         case 3 
            if(inptBits == 0)
                nextState = 1;
            else
                nextState = 3;
            end
    end
end

function outputs = viterbi_outputs(currentState,inptBits)
    switch currentState
        case 0 
            if(inptBits == 0)
                outputs = 0;
            else
                outputs = 3;
            end
         case 1 
            if(inptBits == 0)
                outputs = 3;
            else
                outputs = 0;
            end
         case 2 
            if(inptBits == 0)
                outputs = 1;
            else
                outputs = 2;
            end
         case 3 
            if(inptBits == 0)
                outputs = 2;
            else
                outputs = 1;
            end
    end
end

%---------CONV_HARDWARE_FUNCTION-----------
function codeword = conv_hardware_213(msg_source)
    bit_string_length = length(msg_source);
    s1 = 0; s2 = 0;
    codeword = zeros(1, bit_string_length * 2);
    for i = 1:bit_string_length
        u0 = xor(msg_source(i), s2);
        u1 = xor(xor(msg_source(i), s1), s2);
        s2 = s1;
        s1 = msg_source(i);
        codeword(2*i-1) = u0;
        codeword(2*i) = u1;
    end
end

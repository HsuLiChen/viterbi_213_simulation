clear; clc; close all;

%% 參數設定
M = 4;                  % 4-PAM
k = log2(M);            % 每個符號所含的 bit 數 (4-PAM -> 2 bits/symbol)
numSymbols = 300000;      % 符號數 (可自行調整大小)
numsamp = 4;            % 每個符號區間採樣點數 (矩形脈衝成形)

EsN0_dB = -15:15;         % 模擬的 Es/N0 (dB) 範圍

trellis = poly2trellis(3, [5 7]); 
tb = 12; 

%% eyediagram setting
fs = 1e6;             % 取樣率 1 MHz
Rsym = fs / numsamp;  % 符號率
Tsym = 1 / Rsym;      % 符號週期 (秒)

offset = numsamp/2;   % 這裡示範取“半個符號區間”做對齊

%% 產生隨機位元
% 總 bits 數 = 符號數 * 每符號 bits 數
% txBits = randi([0 1], 1,numSymbols * k);
txBits = load('dataIn.asv', '-ascii');
txBits = txBits(1:8190);
txBits = transpose(txBits);

%% 將 bits 映射 (2 bits -> 1 symbol index)
% 例如：00->0, 01->1, 11->3, 10->2 (這是 4-PAM 最常用的 Gray code)
% ---------------------------------------
% MATLAB 內建的 pammod/pamdemod，可直接使用 'gray' 參數自動完成 Gray mapping
% 若想手動指定 Gray code，可自行撰寫查表。這裡示範直接用內建參數。
% ---------------------------------------
symIdxTx = bi2de(reshape(txBits, k, []).','left-msb'); 
%  => 將 txBits reshape 成 [2 x numSymbols]，再將每 2 bits 轉成 0~3 的十進位符號索引
%  => 'left-msb' 表示最左邊是最高位 (與常見定義一致)
hard_code_data = conv_hardware_213(txBits, length(txBits));
symIdxTx_hard = bi2de(reshape(hard_code_data, k, []).','left-msb');
%% 4-PAM 調變 (帶灰度編碼)
% phaseOffset=0, 'gray' => 使用 Gray Mapping
txSymbols = pammod(symIdxTx, M, 0, 'gray');    % 回傳的調變結果為 4 個星座點（實數）
txSymbols_hard = pammod(symIdxTx_hard, M, 0, 'gray');
%% 矩形脈衝成形 (upsample)
txWave = txSymbols;  % 每個符號複製 numsamp 次，長度 = numSymbols*numsamp
txWave_hard = txSymbols_hard;
% eyediagram(txWave_hard, 2*numsamp, 2*Tsym, offset);
% title('4-PAM Eye Diagram with Symbol Time in seconds');
% xlabel('Time (s)');
% ylabel('Amplitude');
%% 理論計算用：先量測已成形波形的「符號能量(平均)」
% 在「基帶實數 PAM」情況下，可估計每符號平均能量。
% E_s = mean(abs(txSymbols).^2)；對 4-PAM 的預設星座點，也可手動帶入
Es = mean(abs(txSymbols).^2);
Es_hard = mean(abs(txSymbols_hard).^2);
%% 初始化儲存空間
BER = zeros(size(EsN0_dB));
SER = zeros(size(EsN0_dB));
BER_hard = zeros(size(EsN0_dB));
SER_hard = zeros(size(EsN0_dB));
%% 通道模擬：在不同的 Es/N0 下加入 AWGN
for ii = 1:length(EsN0_dB)
    thisEsN0_dB = EsN0_dB(ii);
    thisEsN0    = 10^(thisEsN0_dB/10);      % 線性值

    % -----------------------------
    % *對"實數基帶 PAM"而言*
    % SNR = Es / (N0/2) ==> N0 = 2 * Es / SNR
    % 噪聲(單邊)方差 sigma^2 = N0/2 (因 randn() 預設功率=1)
    % => sigma^2 = (2 * Es / SNR)/2 = Es / SNR
    % => sigma = sqrt( Es / SNR )
    % -----------------------------
    sigma = sqrt( Es / thisEsN0 );
    sigma_hard = sqrt( Es_hard / thisEsN0 );
    % 產生 AWGN 並加到已成形訊號 txWave
    noise = sigma * randn(size(txWave));
    noise_hard = sigma_hard * randn(size(txWave_hard));
    rxWave = txWave + noise;
    rx_wave_hard = txWave_hard + noise_hard;

    % 接收端：整合取樣 (integrate & dump)，對矩形脈衝做簡單匹配
    rxSymbolBlock = rxWave;
    rxSymbolBlock_hard = rx_wave_hard;
    % => 每 numsamp 點加總 (或視情況取平均)，這裡是 sum；可再除以numsamp 做平均也可以，pamdemod 只要相對位置正確即可

    % 解調 (帶灰度解調)
    symIdxRx = pamdemod(rxSymbolBlock, M, 0, 'gray');
    symIdxRx_hard = pamdemod(rxSymbolBlock_hard, M, 0, 'gray');
    
    % 符號錯誤率 (SER) 計算
    [numSymErr_hard, ser_hard] = symerr(symIdxTx_hard, symIdxRx_hard);
    SER_hard(ii) = ser_hard;

    % 符號錯誤率 (SER) 計算
    [numSymErr, ser] = symerr(symIdxTx, symIdxRx);
    SER(ii) = ser;

    % 符號索引 -> bits
    rxBitsMat = de2bi(symIdxRx, k, 'left-msb');   % 轉回 2 bits
    rxBits    = reshape(rxBitsMat.', 1, []);      % [2 x numSymbols] -> [1 x (2*numSymbols)]

    % 符號索引 -> bits
    rxBitsMat_hard = de2bi(symIdxRx_hard, k, 'left-msb');   % 轉回 2 bits
    rxBits_hard    = reshape(rxBitsMat_hard.', 1, []);      % [2 x numSymbols] -> [1 x (2*numSymbols)]
    decoded_hardware = hardware_viterbi213(rxBits_hard, length(txBits));
    decoded_matlab = vitdec(rxBits_hard,trellis,tb, 'trunc' , 'hard' );
    % rxBits_hardV3 = viterbi_hardwareV3_213(rxBits_hard,tb);
    
    for k_tb = 2:2:16
        for k_L = 1:8
            rxBits_hardV3 = viterbi_hardwareV3_213(rxBits_hard,k_tb);
            % 位元錯誤率 (BER) 計算
            [~, ber_HARDV3] = biterr(txBits, rxBits_hardV3);
            BER_HARDV3(k_L,ii) = ber_HARDV3;
        end
    end
    % rxBits_hardV3 = viterbi_hardwareV3_213(rxBits_hard,tb);

    % 位元錯誤率 (BER) 計算
    [~, ber] = biterr(txBits, rxBits);
    BER(ii) = ber;

    % 位元錯誤率 (BER) 計算
    [~, ber_hard] = biterr(txBits, decoded_hardware);
    BER_hard(ii) = ber_hard;

end

%% 4-PAM 理論 SER (參考)
% 4-PAM (Gray code) 的最小歐幾里得距離 d_min = 2*(modLevel間距的一半)
% 但常見教科書給出：SER(4-PAM) = 3/2 * Q( sqrt( (2/5)*Es/N0 ) )  [若符號間距為 2, 平均能量= (1^2 +3^2)/2 ...]
% 或其他係數，依照定義不同會有 slight 誤差。可用下列方式做為一條大致參考曲線：
theorySER = zeros(size(EsN0_dB));
for ii = 1:length(EsN0_dB)
    gamma = 10^(EsN0_dB(ii)/10);  % Es/N0(線性)
    % 一種常見 4-PAM SER 近似公式 (星座點為 ±1, ±3)
    theorySER(ii) = 3/2 * qfunc( sqrt( (2/5) * gamma) );
end


%% 繪圖 (BER / SER)
figure;
semilogy(EsN0_dB, BER, '-o', 'LineWidth', 2, 'DisplayName', 'Uncoded BER'); hold on;
semilogy(EsN0_dB, BER_hard, '-s', 'LineWidth', 2, 'DisplayName', 'Hardware Decoded BER');

% 逐條繪製 software Viterbi 不同 traceback 長度的 BER
[k_max, num_EsN0] = size(BER_HARDV3);
markers = {'-*', '-d', '-^', '-v', '-<', '->', '-p', '-h'}; % 8種不同標記，對應 traceback 長度
for k_tb_idx = 1:k_max
    semilogy(EsN0_dB, BER_HARDV3(k_tb_idx, :), ...
        markers{mod(k_tb_idx-1, length(markers))+1}, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Hardware Viterbi TB=%d', k_tb_idx*2));
end

semilogy(EsN0_dB, theorySER, '--', 'LineWidth', 2, 'DisplayName', 'Theoretical SER');

grid on;
xlabel('E_s/N_0 (dB)');
ylabel('Bit Error Rate (BER)');
legend show; % 顯示所有圖例
title('BER Performance of 4-PAM System with Different Traceback Lengths');

%---------hardware_conv213_function-----------
function codeword = conv_hardware_213(msg_source, bit_string_length)
    s1 = 0;
    s2 = 0;
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

%---------hardware_viterbi213_function-----------
function decoded_bits = hardware_viterbi213(code_data, bit_string_length)
    % 定義狀態轉移和輸出（修正為與硬體編碼器一致）
    trellis = [
        0 0 0 [0 0];   % 狀態0，輸入0 → 狀態0，輸出00
        0 1 2 [1 1];   % 狀態0，輸入1 → 狀態2，輸出11
        1 0 0 [1 1];   % 狀態1，輸入0 → 狀態0，輸出11
        1 1 2 [0 0];   % 狀態1，輸入1 → 狀態2，輸出00
        2 0 1 [0 1];   % 狀態2，輸入0 → 狀態1，輸出01
        2 1 3 [1 0];   % 狀態2，輸入1 → 狀態3，輸出10
        3 0 1 [1 0];   % 狀態3，輸入0 → 狀態1，輸出10
        3 1 3 [0 1];   % 狀態3，輸入1 → 狀態3，輸出01
    ];

    % 初始化路徑度量（強制初始狀態為00）
    num_states = 4;
    path_metrics = inf(1, num_states);
    path_metrics(1) = 0; % 初始狀態為00（索引1對應狀態0）
    path_history = zeros(num_states, bit_string_length); % 列=狀態，行=時間步

    % 解碼過程
    for step = 1:bit_string_length
        idx = 2*step - 1;
        received_bits = code_data(idx:idx+1);
        new_path_metrics = inf(1, num_states);
        temp_history = zeros(1, num_states);

        for current_state = 0:num_states-1
            if path_metrics(current_state + 1) == inf
                continue;
            end

            for input_bit = 0:1
                % 從trellis查找轉移（修正索引計算）
                row = current_state * 2 + input_bit + 1;
                next_state = trellis(row, 3);
                output_bits = trellis(row, 4:5);

                % 計算漢明距離
                hamming_dist = sum(xor(received_bits, output_bits));
                total_metric = path_metrics(current_state + 1) + hamming_dist;

                % 更新路徑度量與歷史
                if total_metric < new_path_metrics(next_state + 1)
                    new_path_metrics(next_state + 1) = total_metric;
                    temp_history(next_state + 1) = current_state;
                end
            end
        end

        path_metrics = new_path_metrics;
        path_history(:, step) = temp_history';
    end

    % 回溯解碼（修正輸入比特提取邏輯）
    decoded_bits = zeros(1, bit_string_length);
    [~, final_state_idx] = min(path_metrics);
    final_state = final_state_idx - 1; % 轉換為0-based狀態

    for step = bit_string_length:-1:1
        % 從path_history取得前驅狀態
        prev_state = path_history(final_state + 1, step);
        
        % 根據前驅狀態與當前狀態的關係反推輸入比特
        % 檢查trellis表：current_state + input_bit → next_state
        input_bit = -1;
        for possible_bit = 0:1
            row = prev_state * 2 + possible_bit + 1;
            if trellis(row, 3) == final_state
                input_bit = possible_bit;
                break;
            end
        end
        
        decoded_bits(step) = input_bit;
        final_state = prev_state;
    end
end

%---------hardwareV3_viterbi213_function-----------
function [decoded_msg] = viterbi_hardwareV3_213(code_msg,D)
    survivors = cell(4,1);       % 儲存各狀態的存活路徑
    new_survivors = cell(4,1);  % 暫存新生成的存活路徑
    path_metrics = [0;3;3;3];   % 各狀態的路徑度量值
    decoded_msg = [];            % 儲存逐步輸出的解碼結果
    conv_code = code_msg;
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
        end
    end
    % 處理剩餘位元（最後D-1個位元）
    [~, final_state] = min(path_metrics);
    remaining_bits = survivors{final_state}(end-D+2:end);
    decoded_msg = [decoded_msg, remaining_bits];
end

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
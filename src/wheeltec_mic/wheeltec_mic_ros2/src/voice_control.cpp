/****************************************************************/
/* Copyright (c) 2023 WHEELTEC Technology, Inc   				*/
/* function:Speech recognition processing						*/
/* 功能：语音识别处理												*/
/****************************************************************/
#include "voice_control.h"

#include <cmath>
#include <iomanip>
#include <limits>
#include <new>
#include <sstream>

int FileSize(const char *fname)
{
    struct stat statbuf;
    if (stat(fname, &statbuf) == 0)
        return statbuf.st_size;
    return -1;
}

/************************************************
Function: Example Initialize recording parameters
功能: 初始化录音参数
*************************************************/
int SpeechProcess::record_params_init(record_handle_t* pcm_handle,record_params_t* params,const char* device_name)
{
	int err;
	unsigned int buffer_time, period_time, sample_rate;

	if (pcm_handle == NULL)
	{
		return -1;
	}

  err = -ENODEV;
  for (int attempt = 0; attempt < audio_open_retries_; ++attempt)
  {
    err = snd_pcm_open(&(pcm_handle->pcm), device_name, SND_PCM_STREAM_CAPTURE, 0);
    if (err >= 0)
    {
      break;
    }
    RCLCPP_WARN(this->get_logger(),
      "Audio device %s is not ready (attempt %d/%d): %s",
      device_name, attempt + 1, audio_open_retries_, snd_strerror(err));
    if (attempt + 1 < audio_open_retries_)
    {
      std::this_thread::sleep_for(
        std::chrono::milliseconds(audio_open_retry_interval_ms_));
    }
  }
  if (err < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to open audio device %s after %d attempts",
      device_name, audio_open_retries_);
    return -1;
  }
	/*参数结构体，可用于指定PCM流的配置*/
	snd_pcm_hw_params_t *hwparams; 

	/*分配硬件参数结构对象，并判断是否分配成功*/
	snd_pcm_hw_params_alloca(&hwparams);

	/*对硬件对象进行初始化默认设置*/
    if((err = snd_pcm_hw_params_any(pcm_handle->pcm,hwparams)) < 0)
    {
    	cout << "初始化参数结构失败:" << "("<< snd_strerror (err) <<")"<<endl;
        goto Init_fail;
    }

 	/*
    	设置数据为交叉模式(降噪板默认输出单通道PCM)
    	INTERLEAVED/NONINTERLEAVED:交叉/非交叉模式。
    	表示在多声道数据传输的过程中是采样交叉的模式还是非交叉的模式。
    	对多声道数据，如果采样交叉模式，使用一块buffer即可，其中各声道的数据交叉传输；
	如果使用非交叉模式，需要为各声道分别分配一个buffer，各声道数据分别传输。
	*/
    if ((err = snd_pcm_hw_params_set_access(pcm_handle->pcm,hwparams,SND_PCM_ACCESS_RW_INTERLEAVED)) < 0)
    {
    	cout << "访问类型设置失败:" << "("<< snd_strerror (err) <<")"<<endl;
        goto Init_fail;
    }

    /*获取格式设置，并设置pcm数据格式*/
    pcm_handle->format = get_formattype_from_params(params);
    if ((err = snd_pcm_hw_params_set_format(pcm_handle->pcm,hwparams,pcm_handle->format)) < 0)
    {
    	cout << "设置PCM数据格式失败:" << "("<< snd_strerror (err) <<")"<<endl;
        goto Init_fail;
    }

    /*获取声道并设置*/
    if ((err = snd_pcm_hw_params_set_channels(pcm_handle->pcm,hwparams,params->channel)) < 0)
    {
    	cout << "channel设置失败:" << "("<< snd_strerror (err) <<")"<<endl;
        goto Init_fail;
    }

    /*获取采样率并设置*/
    sample_rate = params->rate;
	if ((err = snd_pcm_hw_params_set_rate_near(pcm_handle->pcm, hwparams, &sample_rate, 0)) < 0) {
	    cout << "采样率设置失败:" << "(" << snd_strerror(err) << ")" << endl;
	    goto Init_fail;
	}
	pcm_handle->rate = sample_rate;

    //设置周期数
    if ((err = snd_pcm_hw_params_get_buffer_time_max(hwparams,&buffer_time,0)) < 0)
    {
        printf("snd_pcm_hw_params_get_buffer_time_max fail.\n");
        goto Init_fail;
    }
    if (buffer_time > 500000){
        buffer_time = 500000;
    }
    period_time = buffer_time / 4;
    if ((err = snd_pcm_hw_params_set_buffer_time_near(pcm_handle->pcm,hwparams,&buffer_time,0)) < 0)
    {
        printf("snd_pcm_hw_params_set_buffer_time_near fail.\n");
        goto Init_fail;
    }
     if ((err = snd_pcm_hw_params_set_period_time_near(pcm_handle->pcm,hwparams,&period_time,0)) < 0)
    {
        printf("snd_pcm_hw_params_set_period_time_near fail.\n");
        goto Init_fail;
    }

    /*将配置写入驱动程序*/
    if ((err = snd_pcm_hw_params(pcm_handle->pcm,hwparams)) < 0)
    {
    	cout << "写入驱动程序设置参数失败:" << "("<< snd_strerror (err) <<")"<<endl;
        goto Init_fail;
    }

    /*准备音频接口*/
    if ((err = snd_pcm_prepare(pcm_handle->pcm)) < 0)
    {
    	cout << "无法使用音频接口:" << "("<< snd_strerror (err) <<")"<<endl;
        goto Init_fail;
    }

    //获取相关参数
    snd_pcm_uframes_t buffer_size;
    snd_pcm_hw_params_get_period_size(hwparams,&(pcm_handle->chunk_size),0);
    snd_pcm_hw_params_get_buffer_size(hwparams,&buffer_size);
    if (pcm_handle->chunk_size == buffer_size)
     {
        printf("Can't use period equal to buffer size (%lu == %lu).\n",pcm_handle->chunk_size,buffer_size);
        goto Init_fail;
    }

	/*配置一个数据缓冲区用来缓冲数据*/
    pcm_handle->bits_per_sample = snd_pcm_format_width(pcm_handle->format)/8;
    pcm_handle->bits_per_frame = pcm_handle->bits_per_sample*params->channel;
    pcm_handle->chunk_bytes = pcm_handle->chunk_size*pcm_handle->bits_per_frame;
    pcm_handle->buffer = (unsigned char *)malloc(pcm_handle->chunk_bytes);
    if (!pcm_handle->buffer) 
    { 
    	cout << "Error malloc" <<endl;
        goto Init_fail; 
    }
    // cout << "已初始化录音参数" <<endl;
    return 0;

Init_fail: 
	snd_pcm_close(pcm_handle->pcm);
    return -1; 
}

/**************************************
Function: Resample audio using libsamplerate
功能: 重采样音频数据
***************************************/
int SpeechProcess::resample_audio(const short *input, int input_samples, 
                   short **output, int *output_samples, 
                   int channels, double ratio) {
    if (!input || input_samples <= 0 || channels <= 0 || ratio <= 0) {
        printf("Invalid resample parameters\n");
        return -1;
    }
    
    // 添加输入数据检查
    if (input_samples < channels * 100) {  // 至少需要100帧
        printf("Input samples too small: %d\n", input_samples);
        return -1;
    }
    
    SRC_STATE *src_state;
    SRC_DATA src_data;
    int error;
    
    src_state = src_new(SRC_SINC_FASTEST, channels, &error);
    if (!src_state) {
        printf("Error initializing samplerate converter: %s\n", src_strerror(error));
        return -1;
    }
    
    // 更精确的输出缓冲区大小计算
    int max_output_samples = (int)(input_samples * ratio) + channels * 100;
    
    float *input_float = (float*)malloc(input_samples * sizeof(float));
    float *output_float = (float*)malloc(max_output_samples * sizeof(float));
    
    if (!input_float || !output_float) {
        printf("Error allocating float buffers\n");
        free(input_float);
        free(output_float);
        src_delete(src_state);
        return -1;
    }
    
    // 使用更精确的转换
    float scale_factor = 1.0f / 32768.0f;
    for (int i = 0; i < input_samples; i++) {
        input_float[i] = (float)input[i] * scale_factor;
        // 防止溢出
        if (input_float[i] > 1.0f) input_float[i] = 1.0f;
        if (input_float[i] < -1.0f) input_float[i] = -1.0f;
    }
    
    *output = (short*)malloc(max_output_samples * sizeof(short));
    if (!*output) {
        printf("Error allocating output buffer\n");
        free(input_float);
        free(output_float);
        src_delete(src_state);
        return -1;
    }
    
    src_data.data_in = input_float;
    src_data.data_out = output_float;
    src_data.input_frames = input_samples / channels;
    src_data.output_frames = max_output_samples / channels;
    src_data.src_ratio = ratio;
    src_data.end_of_input = 1;  // 标记为输入结束
    
    error = src_process(src_state, &src_data);
    
    if (error) {
        printf("Error during resampling: %s\n", src_strerror(error));
        free(input_float);
        free(output_float);
        free(*output);
        src_delete(src_state);
        return -1;
    }
    
    *output_samples = src_data.output_frames_gen * channels;
    
    // 输出限制
    float out_scale = 32767.0f;
    for (int i = 0; i < *output_samples; i++) {
        float sample = output_float[i] * out_scale;
        if (sample > 32767.0f) sample = 32767.0f;
        if (sample < -32768.0f) sample = -32768.0f;
        (*output)[i] = (short)sample;
    }
    
    // printf("Resampling: %d samples -> %d samples (ratio: %.3f, frames: %ld -> %ld)\n", 
    //        input_samples, *output_samples, ratio,
    //        src_data.input_frames, src_data.output_frames_gen);
    
    free(input_float);
    free(output_float);
    src_delete(src_state);
    
    return 0;
}

/***********************************************
Function: Initialize offline resource parameters
功能: 初始化离线资源参数
************************************************/
int SpeechProcess::init_asr_params(){
	init_rec = 0;
	init_success = 0;
	write_first_data = 0;

    std::string jet_path = BEGIN_PREFIX + source_path + ASR_RES_PATH;
    std::string grammer_path = source_path + GRM_BUILD_PATH;
    std::string bnf_path = source_path + GRM_FILE;
    denoise_sound_path = source_path + DENOISE_SOUND_PATH;

    APPID = const_cast<char *>(appid.c_str());

	Recognise_Result inital = initial_asr_paramers(
		const_cast<char*>(jet_path.c_str()),
		const_cast<char*>(grammer_path.c_str()), 
		const_cast<char*>(bnf_path.c_str()), 
		const_cast<char*>(LEX_NAME.c_str()));
	if (!inital.whether_recognised)
	{
		cout <<"fail_reason :" << inital.fail_reason << endl;
		return -1;
	}
	return 0;
}

/**************************************
Function: Get file size
功能: 获取文件大小
***************************************/
int SpeechProcess::filesize(const char *fname)
{
	struct stat statbuf;
    if (stat(fname, &statbuf) == 0)
        return statbuf.st_size;
    return -1;
}

/**************************************
Function: Text encoding conversion
功能: 文本编码转换
***************************************/

std::string SpeechProcess::s2s(const std::string &str)
{
	using convert_typeX =  std::codecvt_utf8<wchar_t>;
	std::wstring_convert<convert_typeX, wchar_t> converterX;
	std::wstring wstr = converterX.from_bytes(str);
	return converterX.to_bytes(wstr);
}

/**************************************
Function: Audio format selection
功能: 音频格式选择
***************************************/
snd_pcm_format_t SpeechProcess::get_formattype_from_params(record_params_t* params)
{
    if(params!=NULL){  
        switch (params->format) {  
        case 0:
            return SND_PCM_FORMAT_S8;
        case 1:
            return SND_PCM_FORMAT_U8;
        case 2:
            return SND_PCM_FORMAT_S16_LE;
        case 3:
            return SND_PCM_FORMAT_S16_BE;
        default:  return SND_PCM_FORMAT_S16_LE;
        }
    }
    return SND_PCM_FORMAT_S16_LE;
}

/**************************************
Function: Text encoding conversion
功能: 送入音频进行识别
***************************************/
int SpeechProcess::business_data_t(unsigned char* record, size_t bytes_read)
{
    record_data = record;
	if (!init_success && init_rec)
	{
		if (record_data == nullptr || bytes_read == 0) {
			return 0;
		}
		if (bytes_read > std::numeric_limits<unsigned int>::max()) {
			RCLCPP_ERROR(this->get_logger(),
				"PCM block is too large for the ASR API: %zu bytes", bytes_read);
			return -1;
		}

		const unsigned int len = static_cast<unsigned int>(bytes_read);
		std::unique_ptr<char[]> pcm_buffer(new (std::nothrow) char[len]);
		if (!pcm_buffer) {
			cout << ">>>>>buffer is null" << endl;
			return -1;
		}
		memcpy(pcm_buffer.get(), record_data, len);

		if (write_first_data++ == 0) {
#if whether_print_log
        	cout <<"***************write the first voice**********" <<endl;
#endif
			demo_xf_mic(pcm_buffer.get(), len, 1);
		} else {
#if whether_print_log
        	cout <<"***************write the middle voice**********" <<endl;
#endif
			demo_xf_mic(pcm_buffer.get(), len, 2);
		}

		if (whether_finised) {
			record_finish = 1;
			whether_finised = 0;
		}
    }
    return 0;
}

/**************************************
Function: Apply software gain to PCM samples
功能: 对采集到的PCM做软件增益，改善远低幅度语音被判定为无有效声音的问题
***************************************/
void SpeechProcess::apply_pcm_gain(short *samples, int sample_count)
{
    if (samples == nullptr || sample_count <= 0 || pcm_gain_ <= 1.0f) {
        return;
    }

    float gain = pcm_gain_;
    if (gain > 8.0f) gain = 8.0f;
    if (gain < 1.0f) gain = 1.0f;

    for (int i = 0; i < sample_count; ++i) {
        int value = static_cast<int>(samples[i] * gain);
        if (value > 32767) value = 32767;
        if (value < -32768) value = -32768;
        samples[i] = static_cast<short>(value);
    }
}

/**************************************
Function: Prepare one bounded per-utterance PCM path
功能: 为每次识别准备独立且有界的PCM文件
***************************************/
std::string SpeechProcess::prepare_utterance_path()
{
    ++utterance_sequence_;
    const uint64_t slot =
        (utterance_sequence_ - 1) % static_cast<uint64_t>(max_saved_utterances_);

    std::ostringstream path;
    path << source_path << "/audio/utterance_"
         << std::setfill('0') << std::setw(3) << slot << ".pcm";
    current_utterance_path_ = path.str();
    return current_utterance_path_;
}

void SpeechProcess::reset_audio_metrics()
{
    audio_sample_count_ = 0;
    audio_square_sum_ = 0.0L;
    audio_peak_abs_ = 0;
    audio_clipped_samples_ = 0;
}

void SpeechProcess::update_audio_metrics(const short *samples, int sample_count)
{
    if (samples == nullptr || sample_count <= 0) {
        return;
    }

    for (int i = 0; i < sample_count; ++i) {
        const int sample = static_cast<int>(samples[i]);
        const int magnitude = sample == -32768 ? 32768 : std::abs(sample);
        audio_square_sum_ +=
            static_cast<long double>(sample) * static_cast<long double>(sample);
        if (magnitude > audio_peak_abs_) {
            audio_peak_abs_ = magnitude;
        }
        if (magnitude >= 32760) {
            ++audio_clipped_samples_;
        }
    }
    audio_sample_count_ += static_cast<uint64_t>(sample_count);
}

void SpeechProcess::log_audio_metrics(
    const std::string &status, int result_confidence,
    const std::string &recognized_text)
{
    const double sample_count = static_cast<double>(audio_sample_count_);
    const double rms = audio_sample_count_ > 0
        ? std::sqrt(static_cast<double>(audio_square_sum_ / audio_sample_count_))
        : 0.0;
    const double rms_dbfs = rms > 0.0
        ? 20.0 * std::log10(rms / 32768.0)
        : -120.0;
    const double peak_dbfs = audio_peak_abs_ > 0
        ? 20.0 * std::log10(static_cast<double>(audio_peak_abs_) / 32768.0)
        : -120.0;
    const double clip_percent = audio_sample_count_ > 0
        ? 100.0 * static_cast<double>(audio_clipped_samples_) / sample_count
        : 0.0;
    const double duration_s = sample_count / 16000.0;

    RCLCPP_INFO(
        this->get_logger(),
        "ASR_AUDIO utterance=%llu file=%s duration_s=%.3f samples=%llu "
        "rms=%.1f rms_dbfs=%.2f peak=%d peak_dbfs=%.2f clipped=%llu "
        "clip_pct=%.5f status=%s confidence=%d threshold=%d text=%s",
        static_cast<unsigned long long>(utterance_sequence_),
        current_utterance_path_.c_str(), duration_s,
        static_cast<unsigned long long>(audio_sample_count_), rms, rms_dbfs,
        audio_peak_abs_, peak_dbfs,
        static_cast<unsigned long long>(audio_clipped_samples_), clip_percent,
        status.c_str(), result_confidence, confidence, recognized_text.c_str());
}

/**************************************
Function: Get audio data
功能: 获取音频
***************************************/
int SpeechProcess::get_record_sound(const char *fname)
{
    int ret;
    const char *filename = fname;
    int fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR);
    if (fd == -1)
    {
        cout << "无法创建音频文件" <<endl;
        return -1;
    }
    
    // 使用 fdopen 创建 FILE* 指针
    FILE *pcm_file = fdopen(fd, "wb");
    if (!pcm_file) {
        cout << "无法打开音频文件流" << endl;
        close(fd);
        return -1;
    }
    
    init_success = record_params_init(&record, &params, record_device_name.c_str());
    if (init_success != RET_SUCCESS)
    {
        cout << "音频初始化失败!" <<endl;
        fclose(pcm_file);
        return -1;
    }

    // 检查是否需要重采样
    bool need_resample = (record.rate != 16000);
    double ratio = 16000.0 / (double)record.rate;
    // if (need_resample) {
    //     printf("需要重采样: %.6f (%uHz -> 16000Hz)\n", ratio, record.rate);
    // } else {
    //     printf("采样率已是16000Hz，跳过重采样处理\n");
    // }
    cout<<endl;
    cout<<">>>>>开始一次语音识别！"<<endl;
    
	while (init_success == RET_SUCCESS) {
		if (cancel_requested_.load()) {
			recognition_cancelled_ = true;
			RCLCPP_INFO(this->get_logger(),
				"A newer wake event cancelled the active recording window");
			break;
		}

		ret = snd_pcm_readi(record.pcm, record.buffer, record.chunk_size);
        if (ret == -EAGAIN) {
            snd_pcm_wait(record.pcm, 1000);
            continue;
        }
        else if (ret == -EPIPE) {
            snd_pcm_prepare(record.pcm);
            printf("snd_pcm_readi return EPIPE, recovered.\n");
            continue;
        }
        else if (ret == -ESTRPIPE) {
            printf("snd_pcm_readi return ESTRPIPE.\n");
            break;
        }
        else if (ret < 0) {
            printf("snd_pcm_readi return fail: %d\n", ret);
            break;
        }
        else if (static_cast<snd_pcm_uframes_t>(ret) != record.chunk_size) {
            printf("读取数据不完整: %d/%lu\n", ret, record.chunk_size);
        }

		if (ret > 0) {
			if (cancel_requested_.load()) {
				recognition_cancelled_ = true;
				RCLCPP_INFO(this->get_logger(),
					"Cancellation arrived while reading PCM; discard this block");
				break;
			}
			init_rec = 1;

            short *process_data = NULL;
            int process_samples = 0;
            int input_samples = ret * params.channel;

            if (need_resample) {
                if (resample_audio((short*)record.buffer, input_samples, 
                                  &process_data, &process_samples, 
                                  params.channel, ratio) == 0) {
                     //printf("重采样成功: %d -> %d 样本\n", input_samples, resampled_samples);
                    if (process_samples > 0) {
                        apply_pcm_gain(process_data, process_samples);
                        update_audio_metrics(process_data, process_samples);
                        if (save_pcm_local) {
                            (void)fwrite(process_data,
                                         sizeof(short),
                                         process_samples,
                                         pcm_file);
                            fflush(pcm_file);
                        }
                        business_data_t(
                            reinterpret_cast<unsigned char*>(process_data),
                            static_cast<size_t>(process_samples) * sizeof(short));
                    }
                    free(process_data);
                } else {
                    printf("重采样失败，使用原始数据\n");
                    apply_pcm_gain(
                        reinterpret_cast<short *>(record.buffer), input_samples);
                    update_audio_metrics(
                        reinterpret_cast<short *>(record.buffer), input_samples);
                    if (save_pcm_local) {
                        fwrite(record.buffer, record.bits_per_sample, ret * params.channel, pcm_file);
                    }
                    business_data_t(record.buffer, static_cast<size_t>(ret) * record.bits_per_frame);
                }
            } else {
                // 无需重采样，直接使用原始数据
                process_data = (short*)record.buffer;
                process_samples = input_samples;
                apply_pcm_gain(process_data, process_samples);
                update_audio_metrics(process_data, process_samples);
                
                if (save_pcm_local) {
                    (void)fwrite(process_data,
                                 sizeof(short),
                                 process_samples,
                                 pcm_file);
                    fflush(pcm_file);
                }
                business_data_t(
                    reinterpret_cast<unsigned char*>(process_data),
                    static_cast<size_t>(process_samples) * sizeof(short));
            }
        }
        if (record_finish) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    init_rec = 0;
    fclose(pcm_file);
    finish_record_sound();
    return 0;
}

/**************************************
Function: Finish recording audio
功能: 结束录制音频
***************************************/
int SpeechProcess::finish_record_sound()
{
	if(record.buffer != NULL) free(record.buffer);
    if(!init_success) snd_pcm_close(record.pcm);
    printf(">>>>>停止录音........\n"); 
    return 0;
}

Effective_Result SpeechProcess::show_result(char *str)
{
    Effective_Result current;
    current.effective_confidence = 0;
    strcpy(current.effective_word, " ");
    
    // 添加空指针和长度检查
    if (str == nullptr || strlen(str) < 50) {
        return current;
    }

    std::string result_str(str);
    size_t rawtext_start = result_str.find("<rawtext>");
    size_t rawtext_end = result_str.find("</rawtext>");
    size_t confidence_start = result_str.find("<confidence>");
    size_t confidence_end = result_str.find("</confidence>");
 
 	// 检查所有标签是否都存在   
    if (rawtext_start == std::string::npos || rawtext_end == std::string::npos ||
        confidence_start == std::string::npos || confidence_end == std::string::npos) {
        return current;
    }

    // 提取置信度
    if (confidence_start + 12 < confidence_end) {
        std::string conf_str = result_str.substr(confidence_start + 12, confidence_end - (confidence_start + 12));
        try {
            current.effective_confidence = std::stoi(conf_str);
        } catch (...) {
            current.effective_confidence = 0;
        }
    }

    // 提取识别结果
    if (current.effective_confidence >= confidence && 
        rawtext_start + 9 < rawtext_end) {
        std::string word_str = result_str.substr(rawtext_start + 9, rawtext_end - (rawtext_start + 9));
        if (word_str.length() < sizeof(current.effective_word)) {
            strncpy(current.effective_word, word_str.c_str(), sizeof(current.effective_word)-1);
            current.effective_word[sizeof(current.effective_word)-1] = '\0';
        }
    }
    return current;
}

/********************************************************
Function: Get the offline command word recognition result
功能: 获取离线命令词识别结果
*********************************************************/
bool SpeechProcess::Get_Offline_Recognise_Result(const std::shared_ptr<wheeltec_mic_msg::srv::GetOfflineResult::Request>& request,
							std::shared_ptr<wheeltec_mic_msg::srv::GetOfflineResult::Response>& response){
	std::lock_guard<std::mutex> recognition_lock(recognition_mutex_);
	if (request->offline_recognise_start)
	{
		// call_recognition sends this service request only after node_feedback
		// has confirmed that awake.wav and the 200 ms speaker tail are complete.
		// A cancel may arrive before the old request reaches this callback.
		if (cancel_requested_.exchange(false)) {
			response->result = "fail";
			response->fail_reason = "cancelled_by_rewake";
			response->text = " ";
			RCLCPP_INFO(this->get_logger(),
				"Discard recognition request cancelled before recording started");
			return true;
		}
		recognition_cancelled_ = false;
		whether_finised = 0;
		record_finish = 0;
		time_per_order = request->time_per_order;
		confidence = request->confidence_threshold;
		reset_audio_metrics();
		prepare_utterance_path();
		int ret = create_asr_engine(&asr_data);
		if (MSP_SUCCESS != ret)
		{
			cout<<"创建语音识别引擎失败！"<<endl;
			log_audio_metrics("engine_create_failed", 0, "");
			return false;
		}

		if (get_record_sound(current_utterance_path_.c_str()) != RET_SUCCESS)
          {
                  response->result = "fail";
                  response->fail_reason = "audio_device_unavailable";
                  response->text = " ";
                  RCLCPP_ERROR(this->get_logger(),
                          "Offline recognition skipped because audio device %s is unavailable",
                          record_device_name.c_str());
                  log_audio_metrics("audio_device_unavailable", 0, "");
                  delete_asr_engine();
                  write_first_data = 0;
                  return true;
          }

		if (recognition_cancelled_.load() || cancel_requested_.exchange(false)) {
			response->result = "fail";
			response->fail_reason = "cancelled_by_rewake";
			response->text = " ";
			log_audio_metrics("cancelled_by_rewake", 0, "");
			whole_result = const_cast<char*>("");
			delete_asr_engine();
			write_first_data = 0;
			recognition_cancelled_ = false;
			RCLCPP_INFO(this->get_logger(),
				"Recognition window cancelled; suppress its result");
			return true;
		}

        if (whole_result != nullptr && whole_result[0] != '\0')
		{
			Effective_Result effective_ans = show_result(whole_result);
			if (effective_ans.effective_confidence >= confidence)
			{
				cout<<">>>>>是否识别成功: 是 " <<endl;
				cout<<">>>>>关键字的置信度: [" << effective_ans.effective_confidence << "] " <<endl;
				cout<<">>>>>关键字识别结果: [" << effective_ans.effective_word << "] " <<endl;

				response->result = "ok";
				response->fail_reason = "";
				std::string txt_uft8 = s2s(effective_ans.effective_word);
				response->text = txt_uft8;
				log_audio_metrics(
				    "ok", effective_ans.effective_confidence, txt_uft8);

				std_msgs::msg::String msg;
				msg.data = effective_ans.effective_word;
				voice_words_pub->publish(msg);			
			}
			else
			{
				cout<<">>>>>是否识别成功: 否 " <<endl;
				cout<<">>>>>关键字的置信度: [" << effective_ans.effective_confidence << "] " <<endl;
				cout<<">>>>>关键字置信度较低，文本不予显示" <<endl;

				response->result = "fail";
				response->fail_reason = "low_confidence error or 11212_license_expired_error";
				response->text = " ";
				log_audio_metrics(
				    "low_confidence", effective_ans.effective_confidence, "");
			}
		}
		else
		{
			response->result = "fail";
			response->fail_reason = "no_valid_sound error";
			response->text = " ";
			log_audio_metrics("no_valid_sound", 0, "");
			cout<<">>>>>未能检测到有效声音,请重试" <<endl;
		}
        whole_result = const_cast<char*>("");
		/*[1-3]语音识别结束]*/
		delete_asr_engine();
		write_first_data = 0;
	}
	cout<<endl;
	return true;
}

void SpeechProcess::run()
{
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(shared_from_this());
    executor.spin();
}

void SpeechProcess::recognition_cancel_Callback(
	const std_msgs::msg::UInt64::SharedPtr msg)
{
	cancel_requested_ = true;
	RCLCPP_INFO(this->get_logger(),
		"Recognition cancellation requested by wake generation %llu",
		static_cast<unsigned long long>(msg->data));
}

SpeechProcess::SpeechProcess(const std::string &node_name) 
: rclcpp::Node(node_name){
	/***声明参数并获取***/
	this->declare_parameter<string>("appid","5fa0b8b9");
	this->get_parameter("appid",appid);
	this->declare_parameter<string>("record_device_name", "default");
	this->get_parameter("record_device_name", record_device_name);
	this->declare_parameter<std::string>("device_type", "default");
	this->get_parameter("device_type", device_type);
  this->declare_parameter<int>("audio_open_retries", 1);
  this->declare_parameter<int>("audio_open_retry_interval_ms", 1000);
  this->get_parameter("audio_open_retries", audio_open_retries_);
  this->get_parameter("audio_open_retry_interval_ms", audio_open_retry_interval_ms_);
  this->declare_parameter<int>("max_saved_utterances", 100);
  this->get_parameter("max_saved_utterances", max_saved_utterances_);
  if (max_saved_utterances_ < 1) {
    max_saved_utterances_ = 1;
  }
  if (max_saved_utterances_ > 1000) {
    max_saved_utterances_ = 1000;
  }
  if (audio_open_retries_ < 1) {
    audio_open_retries_ = 1;
  }
  if (audio_open_retry_interval_ms_ < 100) {
    audio_open_retry_interval_ms_ = 100;
  }
	this->declare_parameter<double>("pcm_gain", 2.0);
	double pcm_gain_param = 2.0;
	this->get_parameter("pcm_gain", pcm_gain_param);
	if (pcm_gain_param < 1.0) {
		pcm_gain_param = 1.0;
	}
	if (pcm_gain_param > 8.0) {
		pcm_gain_param = 8.0;
	}
	pcm_gain_ = static_cast<float>(pcm_gain_param);
	RCLCPP_INFO(this->get_logger(),
    "voice_control device=%s pcm_gain=%.2f max_saved_utterances=%d "
    "audio_open_retries=%d retry_interval_ms=%d",
    record_device_name.c_str(), pcm_gain_, max_saved_utterances_,
    audio_open_retries_, audio_open_retry_interval_ms_);
	/***识别命令词话题发布者创建***/
	voice_words_pub = this->create_publisher<std_msgs::msg::String>("voice_words",10);

	recognition_cancel_callback_group_ =
		this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
	rclcpp::SubscriptionOptions cancel_options;
	cancel_options.callback_group = recognition_cancel_callback_group_;
	recognition_cancel_sub_ = this->create_subscription<std_msgs::msg::UInt64>(
		"recognition_cancel", 10,
		std::bind(
			&SpeechProcess::recognition_cancel_Callback, this,
			std::placeholders::_1),
		cancel_options);

	get_offline_result_srv_ = this->create_service<wheeltec_mic_msg::srv::GetOfflineResult>(
		"get_offline_result_srv",[this](const std::shared_ptr<wheeltec_mic_msg::srv::GetOfflineResult::Request> request,
									std::shared_ptr<wheeltec_mic_msg::srv::GetOfflineResult::Response> response){
									Get_Offline_Recognise_Result(request,response);
		});

	int ret = init_asr_params();
	if(ret == RET_SUCCESS)
	{
		RCLCPP_INFO(this->get_logger(),"Initialization Offline resource parameter success!");
	}

    g_play_state::init();

	timeout_thread_ = std::thread(&SpeechProcess::timeoutCheckLoop, this);
}

/********************************************************
Function: Calculates whether recording times out
功能: 检测录音是否超时
*********************************************************/
void SpeechProcess::timeoutCheckLoop()
{
    rclcpp::Time start_time;
    bool recording = false;
    
    while(!stop_timeout_check_) {   
        if (init_rec && !recording) {
            start_time = rclcpp::Node::now();
            recording = true;
        }
        
        if (recording && init_rec && whether_finised != 1) {
            auto current_time = rclcpp::Node::now();
            if ((current_time - start_time).seconds() > time_per_order) {
                cout << ">>>>>>超出离线命令词最长识别时间" << endl;
                whether_finised = 1;
                recording = false;
                record_finish = 1;
            }
        } else if (!init_rec) {
            recording = false;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

SpeechProcess::~SpeechProcess()
{
    stop_timeout_check_ = true;
    if (timeout_thread_.joinable()) {
        timeout_thread_.join();
    }

	RCLCPP_INFO(this->get_logger(),"voice_control node over!\n");
}

// void exit_sighandler(int sig)
// {
// 	record_finish = 1;
// }

int main(int argc, char **argv)
{
	rclcpp::init(argc,argv);
	// /*注册信号捕获退出接口*/
	// signal(2,exit_sighandler);
    atexit([]() {
        SharedMemory::cleanup();
    });
    auto node = std::make_shared<SpeechProcess>("voice_control");
    node->run(); 
  	rclcpp::shutdown();
	return 0;
} 

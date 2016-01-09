#include "audio.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "misc.h"
#include <math.h>
#include "ff.h"
#include "stm32f4xx_dac.h"
#include "mp3dec.h"
#include "mp3common.h"
#include "gui.h"
#include "stm32f4_discovery_audio_codec.h"
#include "stm32f4xx_spi.h"

/*move from waveplayer*/
#ifdef I2S_24BIT
extern uint16_t sampleBuffer[((48*8) * 200) / 2];	//sample frequency (1 packet per ms) times format (bytes)
#else
extern uint16_t sampleBuffer[((48*4) * 300) / 2];	//sample frequency (1 packet per ms) times format (bytes)
#endif
extern int inCurIndex;

//use just the minimum needed
__IO uint8_t volume = 80;
__IO uint8_t AudioPlayStart = 0;
static __IO uint32_t TimingDelay;
uint8_t Buffer[6];
static void Mems_Config(void);

#if 0
/** @addtogroup STM32F4-Discovery_Audio_Player_Recorder
* @{
*/ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#if defined MEDIA_IntFLASH
 /* This is an audio file stored in the Flash memory as a constant table of 16-bit data.
    The audio format should be WAV (raw / PCM) 16-bits, Stereo (sampling rate may be modified) */
extern uint16_t AUDIO_SAMPLE[];
/* Audio file size and start address are defined here since the audio file is 
    stored in Flash memory as a constant table of 16-bit data */
#define AUDIO_FILE_SZE          990000
#define AUDIO_START_ADDRESS     58 /* Offset relative to audio file header size */
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if defined MEDIA_USB_KEY
 extern __IO uint8_t Command_index;
 static uint32_t wavelen = 0;
 static char* WaveFileName ;
 static __IO uint32_t SpeechDataOffset = 0x00;
 __IO ErrorCode WaveFileStatus = Unvalid_RIFF_ID;
 UINT BytesRead;
 WAVE_FormatTypeDef WAVE_Format;
 uint16_t buffer1[_MAX_SS] ={0x00};
 uint16_t buffer2[_MAX_SS] ={0x00};
 uint8_t buffer_switch = 1;
 extern FATFS fatfs;
 extern FIL file;
 extern FIL fileR;
 extern DIR dir;
 extern FILINFO fno;
 extern uint16_t *CurrentPos;
 extern USB_OTG_CORE_HANDLE USB_OTG_Core;
 extern uint8_t WaveRecStatus;
#endif

__IO uint32_t XferCplt = 0;
__IO uint32_t WaveCounter;
__IO uint32_t WaveDataLength = 0;
extern __IO uint8_t Count;
extern __IO uint8_t RepeatState ;
extern __IO uint8_t LED_Toggle;
extern __IO uint8_t PauseResumeStatus ;
extern uint32_t AudioRemSize; 

/* Private function prototypes -----------------------------------------------*/
#if 0
static void EXTILine_Config(void);
#endif
#if defined MEDIA_USB_KEY
static ErrorCode WavePlayer_WaveParsing(uint32_t *FileLen);
#endif

/* Private functions ---------------------------------------------------------*/

/*move from waveplayer*/

uint32_t play_time_other = 0;
uint32_t play_time_sec = 0;

#define BUF_LENGTH 2304
#define READBUF_SIZE 2000

int i;

uint16_t *buf;
uint16_t *buf2;
static uint8_t readBuf[READBUF_SIZE];
int chunck2_size = 0;
uint8_t cur_buf = 0;
uint8_t update_buf = 0;
uint32_t cur_point = 0;

FIL fil;
static HMP3Decoder hMP3Decoder;
MP3FrameInfo mp3FrameInfo; 
volatile u32 bytesLeft = 0; 
volatile u32 outOfData = 0;
uint8_t *readPtr = readBuf;
uint32_t offset;  
UINT cnt;
FRESULT fr;
int count = 0;
int ok = 1;
int ismp3 = 0;


void play_test(void *p){
    play("C128.mp3");
}

int play(char * name)
{
    /*if (f_open(&fil, name, FA_OPEN_ALWAYS | FA_READ) != FR_OK) {
        return -1;
    }*/

    if (strstr(name, "MP3")) ismp3 = 1;
    else ismp3 = 0;

    for(int i=0;i<BUF_LENGTH;i++)
    {
        buf[i] = 2000;
        buf2[i] = 2000;
    }

	WavePlayerStart(name);
	
    /*TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM2, ENABLE);*/
    //DAC_Cmd(DAC_Channel_1, ENABLE);
    //DAC_Cmd(DAC_Channel_2, ENABLE);
    /*int cnt;
    f_read(&fil,readBuf,sizeof(readBuf),&cnt);
    bytesLeft += cnt;
    readPtr = readBuf;*/

    return 0;
}

void stop()
{
    TIM_Cmd(TIM1, DISABLE);
    TIM_Cmd(TIM2, DISABLE);
    //DAC_Cmd(DAC_Channel_1, DISABLE);
    //DAC_Cmd(DAC_Channel_2, DISABLE);

    play_time_other = 0;
    play_time_sec = 0;
    bytesLeft = 0;
    outOfData = 0;

    f_close(&fil);
}

void player_init(void)
{
    //PWM_RCC_Configuration();
    //DAC_Configuration();
	AudioPlayStart = 0;
	RepeatState =0;
	
	WavePlayerInit(I2S_AudioFreq_48k);
    hMP3Decoder = MP3InitDecoder();

    /*PWM_GPIO_Configuration();
    PWM_TIM2_Configuration();
    PWM_TIM1_Configuration();*/

    buf = pvPortMalloc(sizeof(uint16_t)*BUF_LENGTH);
    buf2 = pvPortMalloc(sizeof(uint16_t)*BUF_LENGTH);
}

void  PWM_RCC_Configuration(void)
{
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE );//Enalbe AHB for GPIOB
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1, ENABLE );//Enable APB for TIM1
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );//Enable APB for TIM2
}


void TIM1_UP_TIM10_IRQHandler(void)
{
    TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);

    // if(cur_point == 0)
    // {
    //     if(cur_buf == 1 && ok == 0) {
    //         char ch[12];
    //         sprintf(ch,"%d",++count);
    //         LCD_ClearLine( LINE(10) );
    //         LCD_DisplayStringLine(LINE(10), ch);
    //     }
    // }
    if(cur_buf == 0) 
    {

        TIM1->CCR1 = buf[cur_point*2];
        TIM1->CCR2 = buf[cur_point*2+1];

        // if(cur_point%16 == 0)
        // {
        //     // DAC_SetChannel1Data(DAC_Align_12b_R,data1);
        //     DAC_SetChannel2Data(DAC_Align_12b_R,data2);
        // } 
        // else 
        // {
            
        //     if(cur_point%16 == 15 - buf[(cur_point/16)*2+1] % 16)
        //     {
        //         if(data2 < 4094)
        //         {
        //             ++data2;
        //             DAC_SetChannel1Data(DAC_Align_12b_R,data2);
        //         }
                
        //     }
        // }
    }
    else
    {
        TIM1->CCR1 = buf2[cur_point*2];
        TIM1->CCR2 = buf2[cur_point*2+1];


        // if(cur_point%16 == 0)
        // {
            
        //     // DAC_SetChannel1Data(DAC_Align_12b_R,data1);
        //     DAC_SetChannel2Data(DAC_Align_12b_R,data2);
        // } 
        // else 
        // {
        //     if(cur_point%16 == 15 - buf2[(cur_point/16)*2+1] % 16)
        //     {
        //         if(data2 < 4094)
        //         {
        //             ++data2;
        //             DAC_SetChannel2Data(DAC_Align_12b_R,data2);
        //         }
        //     }
        // }
        
    }

    ++cur_point;
    if(++play_time_other == 44100)
    {
        play_time_sec++;
        play_time_other = 0;
    }

    if(cur_point == BUF_LENGTH / 2)  {
        if(cur_buf == 1)ok = 0;
        cur_buf = !cur_buf;
        update_buf = 1;
        cur_point = 0;
    }
}

void decode(uint16_t *outbuf)
{
    offset = MP3FindSyncWord(readPtr, READBUF_SIZE);
    
    if(offset < 0)
    {
        WavePlayerStop();
    }
    else
    {
        readPtr += offset;                         //data start point
        bytesLeft -= offset;                 //in buffer
        MP3Decode(hMP3Decoder, &readPtr, &bytesLeft, outbuf, 0);

        if (bytesLeft < READBUF_SIZE)
        {
            memmove(readBuf,readPtr,bytesLeft);
            fr = f_read(&fil, readBuf + bytesLeft, READBUF_SIZE - bytesLeft, &cnt);
            if (cnt < READBUF_SIZE - bytesLeft);
                memset(readBuf + bytesLeft + cnt, 0, READBUF_SIZE - bytesLeft - cnt);
            bytesLeft=READBUF_SIZE;
            readPtr=readBuf;               
            if(fr || cnt == 0){
                WavePlayerStop();
            }
        }

        //MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);
        //if(mp3FrameInfo.outputSamps != 2304)   decode(outbuf);
    }
}

void TIM2_IRQHandler(void)
{
    if(update_buf == 1)
    {
        if(cur_buf == 0)   
        {
ok = 1;
            if(ismp3)decode(buf2);
            else    {
                fr = f_read(&fil, buf2, BUF_LENGTH*sizeof(uint16_t), &cnt);
                if (fr || cnt == 0){
                    stop();
                    next_song();
                }
            }
            int i;
            for(i=0;i<BUF_LENGTH;i++)
            {
                buf2[i] = ((int)((short)buf2[i])+32768)* 4000 / 65536;
            }

        }
        else
        {
            if(ismp3)decode(buf);
            else { 
                fr = f_read(&fil, buf, BUF_LENGTH*sizeof(uint16_t), &cnt);
                if (fr || cnt == 0){
                    stop();
                    next_song();
                }
            }
            int i;
            for(i=0;i<BUF_LENGTH;i++)
            {
                buf[i] = ((int)((short)buf[i])+32768)* 4000 / 65536;
            }          
        }
        update_buf = 0;
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
}

void  PWM_TIM2_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

  /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 500-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;     
    TIM_OCInitStructure.TIM_Pulse = 99; 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);

    TIM_ARRPreloadConfig(TIM2, ENABLE);

    TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

 /* TIM2 enable counter */
    TIM_Cmd(TIM2, DISABLE);
}

void  PWM_TIM1_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    //TIM_DeInit(TIM1);

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

  /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 3999;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;

    TIM_OCInitStructure.TIM_Pulse = 3999;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset; 

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

 /* TIM4 enable counter */
    TIM_Cmd(TIM1, DISABLE);
}

void PWM_GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM1);
}


/*move from waveplayer*/
/**
* @brief  Initializes the wave player
* @param  AudioFreq: Audio sampling frequency
* @retval None
*/
int WavePlayerInit(uint32_t AudioFreq)
{ 
#if 1
  /* MEMS Accelerometer configure to manage PAUSE, RESUME and Control Volume operation */
  Mems_Config();
#endif

  /* Initialize I2S interface */  
  EVAL_AUDIO_SetAudioInterface(AUDIO_INTERFACE_I2S);
  
  /* Initialize the Audio codec and all related peripherals (I2S, I2C, IOExpander, IOs...) */  
  EVAL_AUDIO_Init(OUTPUT_DEVICE_AUTO, volume, AudioFreq );  
  
  return 0;
}

**
  * @brief  Pause or Resume a played wave
  * @param  state: if it is equal to 0 pause Playing else resume playing
  * @retval None
  */
void WavePlayerPauseResume(uint8_t state)
{ 
  EVAL_AUDIO_PauseResume(state);   
}

/**
  * @brief  Configure the volune
  * @param  vol: volume value
  * @retval None
  */
uint8_t WaveplayerCtrlVolume(uint8_t vol)
{ 
  EVAL_AUDIO_VolumeCtl(vol);
  return 0;
}


/**
  * @brief  Stop playing wave
  * @param  None
  * @retval None
  */
void WavePlayerStop(void)
{ 
  EVAL_AUDIO_Stop(CODEC_PDWN_SW);
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;
  
  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

/**
  * @brief  Reset the wave player
  * @param  None
  * @retval None
  */
void WavePlayer_CallBack(void)
{
  /* Reset the wave player variables */
  RepeatState = 0;
  AudioPlayStart =0;
  LED_Toggle = 7;
  PauseResumeStatus = 1;
  WaveDataLength =0;
  Count = 0;
  
  /* Stops the codec */
  EVAL_AUDIO_Stop(CODEC_PDWN_HW);
  /* LED off */
  STM_EVAL_LEDOff(LED3);
  STM_EVAL_LEDOff(LED4);
  STM_EVAL_LEDOff(LED6);
  
  /* TIM Interrupts disable */
  TIM_ITConfig(TIM4, TIM_IT_CC1, DISABLE);
  f_mount(0, NULL);
} 

/**
  * @brief  Checks the format of the .WAV file and gets information about
  *   the audio format. This is done by reading the value of a
  *   number of parameters stored in the file header and comparing
  *   these to the values expected authenticates the format of a
  *   standard .WAV  file (44 bytes will be read). If  it is a valid
  *   .WAV file format, it continues reading the header to determine
  *   the  audio format such as the sample rate and the sampled data
  *   size. If the audio format is supported by this application,
  *   it retrieves the audio format in WAVE_Format structure and
  *   returns a zero value. Otherwise the function fails and the
  *   return value is nonzero.In this case, the return value specifies
  *   the cause of  the function fails. The error codes that can be
  *   returned by this function are declared in the header file.
  * @param  None
  * @retval Zero value if the function succeed, otherwise it return
  *         a nonzero value which specifies the error code.
  */
static ErrorCode WavePlayer_WaveParsing(uint32_t *FileLen)
{
  uint32_t temp = 0x00;
  uint32_t extraformatbytes = 0;
  
  /* Read chunkID, must be 'RIFF' */
  temp = ReadUnit((uint8_t*)buffer1, 0, 4, BigEndian);
  if (temp != CHUNK_ID)
  {
    return(Unvalid_RIFF_ID);
  }
  
  /* Read the file length */
  WAVE_Format.RIFFchunksize = ReadUnit((uint8_t*)buffer1, 4, 4, LittleEndian);
  
  /* Read the file format, must be 'WAVE' */
  temp = ReadUnit((uint8_t*)buffer1, 8, 4, BigEndian);
  if (temp != FILE_FORMAT)
  {
    return(Unvalid_WAVE_Format);
  }
  
  /* Read the format chunk, must be'fmt ' */
  temp = ReadUnit((uint8_t*)buffer1, 12, 4, BigEndian);
  if (temp != FORMAT_ID)
  {
    return(Unvalid_FormatChunk_ID);
  }
  /* Read the length of the 'fmt' data, must be 0x10 -------------------------*/
  temp = ReadUnit((uint8_t*)buffer1, 16, 4, LittleEndian);
  if (temp != 0x10)
  {
    extraformatbytes = 1;
  }
  /* Read the audio format, must be 0x01 (PCM) */
  WAVE_Format.FormatTag = ReadUnit((uint8_t*)buffer1, 20, 2, LittleEndian);
  if (WAVE_Format.FormatTag != WAVE_FORMAT_PCM)
  {
    return(Unsupporetd_FormatTag);
  }
  
  /* Read the number of channels, must be 0x01 (Mono) or 0x02 (Stereo) */
  WAVE_Format.NumChannels = ReadUnit((uint8_t*)buffer1, 22, 2, LittleEndian);
  
  /* Read the Sample Rate */
  WAVE_Format.SampleRate = ReadUnit((uint8_t*)buffer1, 24, 4, LittleEndian);

  /* Read the Byte Rate */
  WAVE_Format.ByteRate = ReadUnit((uint8_t*)buffer1, 28, 4, LittleEndian);
  
  /* Read the block alignment */
  WAVE_Format.BlockAlign = ReadUnit((uint8_t*)buffer1, 32, 2, LittleEndian);
  
  /* Read the number of bits per sample */
  WAVE_Format.BitsPerSample = ReadUnit((uint8_t*)buffer1, 34, 2, LittleEndian);
  if (WAVE_Format.BitsPerSample != BITS_PER_SAMPLE_16) 
  {
    return(Unsupporetd_Bits_Per_Sample);
  }
  SpeechDataOffset = 36;
  /* If there is Extra format bytes, these bytes will be defined in "Fact Chunk" */
  if (extraformatbytes == 1)
  {
    /* Read th Extra format bytes, must be 0x00 */
    temp = ReadUnit((uint8_t*)buffer1, 36, 2, LittleEndian);
    if (temp != 0x00)
    {
      return(Unsupporetd_ExtraFormatBytes);
    }
    /* Read the Fact chunk, must be 'fact' */
    temp = ReadUnit((uint8_t*)buffer1, 38, 4, BigEndian);
    if (temp != FACT_ID)
    {
      return(Unvalid_FactChunk_ID);
    }
    /* Read Fact chunk data Size */
    temp = ReadUnit((uint8_t*)buffer1, 42, 4, LittleEndian);
    
    SpeechDataOffset += 10 + temp;
  }
  /* Read the Data chunk, must be 'data' */
  temp = ReadUnit((uint8_t*)buffer1, SpeechDataOffset, 4, BigEndian);
  SpeechDataOffset += 4;
  if (temp != DATA_ID)
  {
    return(Unvalid_DataChunk_ID);
  }
  
  /* Read the number of sample data */
  WAVE_Format.DataSize = ReadUnit((uint8_t*)buffer1, SpeechDataOffset, 4, LittleEndian);
  SpeechDataOffset += 4;
  WaveCounter =  SpeechDataOffset;
  return(Valid_WAVE_File);
}

/**
* @brief  Reads a number of bytes from the SPI Flash and reorder them in Big
*         or little endian.
* @param  NbrOfBytes: number of bytes to read.
*         This parameter must be a number between 1 and 4.
* @param  ReadAddr: external memory address to read from.
* @param  Endians: specifies the bytes endianness.
*         This parameter can be one of the following values:
*             - LittleEndian
*             - BigEndian
* @retval Bytes read from the SPI Flash.
*/
uint32_t ReadUnit(uint8_t *buffer, uint8_t idx, uint8_t NbrOfBytes, Endianness BytesFormat)
{
  uint32_t index = 0;
  uint32_t temp = 0;
  
  for (index = 0; index < NbrOfBytes; index++)
  {
    temp |= buffer[idx + index] << (index * 8);
  }
  
  if (BytesFormat == BigEndian)
  {
    temp = __REV(temp);
  }
  return temp;
}

void WavePlayBack(uint32_t AudioFreq,char *name)
{ 
  /* 
  Normal mode description:
  Start playing the audio file (using DMA stream) .
  Using this mode, the application can run other tasks in parallel since 
  the DMA is handling the Audio Transfer instead of the CPU.
  The only task remaining for the CPU will be the management of the DMA 
  Transfer Complete interrupt or the Half Transfer Complete interrupt in 
  order to load again the buffer and to calculate the remaining data.  
  Circular mode description:
  Start playing the file from a circular buffer, once the DMA is enabled it 
  always run. User has to fill periodically the buffer with the audio data 
  using Transfer complete and/or half transfer complete interrupts callbacks 
  (EVAL_AUDIO_TransferComplete_CallBack() or EVAL_AUDIO_HalfTransfer_CallBack()...
  In this case the audio data file is smaller than the DMA max buffer 
  size 65535 so there is no need to load buffer continuously or manage the 
  transfer complete or Half transfer interrupts callbacks. */  
  
  /* Start playing */
  AudioPlayStart = 1;
  RepeatState =0;
  /* Initialize wave player (Codec, DMA, I2C) */
  WavePlayerInit(AudioFreq);

  AudioRemSize = 0; 

  /* Get Data from USB Key */
  f_lseek(&fileR, WaveCounter);
  f_read (&fileR, buffer1, _MAX_SS, &BytesRead); 
  f_read (&fileR, buffer2, _MAX_SS, &BytesRead);
 
  /* Start playing wave */
  Audio_MAL_Play((uint32_t)buffer1, _MAX_SS);
  buffer_switch = 1;
  XferCplt = 0;
  LED_Toggle = 6;
  PauseResumeStatus = 1;
  Count = 0;
 
  while(WaveDataLength != 0)
  { 
    /* Test on the command: Playing */
    if (Command_index == 0)
    { 
      /* wait for DMA transfer complete */
      while(XferCplt == 0)
      {
        if (PauseResumeStatus == 0)
        {
          /* Pause Playing wave */
          LED_Toggle = 0;
          WavePlayerPauseResume(PauseResumeStatus);
          PauseResumeStatus = 2;
        }
        else if (PauseResumeStatus == 1)
        {
          LED_Toggle = 6;
          /* Resume Playing wave */
          WavePlayerPauseResume(PauseResumeStatus);
          PauseResumeStatus = 2;
        }  
      }
      XferCplt = 0;

      if(buffer_switch == 0)
      {
        /* Play data from buffer1 */
        Audio_MAL_Play((uint32_t)buffer1, _MAX_SS);
        /* Store data in buffer2 */
        f_read (&fileR, buffer2, _MAX_SS, &cnt);
		if(ismp3)
			decode(buffer2);
        buffer_switch = 1;
      }
      else 
      { 
		/* Play data from buffer2 */
        Audio_MAL_Play((uint32_t)buffer2, _MAX_SS);
        /* Store data in buffer1 */
        f_read (&fileR, buffer1, _MAX_SS, &cnt);
		if(ismp3)
			decode(buffer1);
        buffer_switch = 0;
      } 
    }
    else 
    {
      WavePlayerStop();
      WaveDataLength = 0;
      RepeatState =0;
      break;
    }
  }
#if defined PLAY_REPEAT_OFF 
  RepeatState = 1;
  WavePlayerStop();
  if (Command_index == 0)
    LED_Toggle = 4;
#else 
  LED_Toggle = 7;
  RepeatState = 0;
  AudioPlayStart = 0;
  WavePlayerStop();
#endif

}


/**
  * @brief  Start wave player
  * @param  None
  * @retval None
  */
int WavePlayerStart(char *fname)
{
  buffer_switch = 1;
  
  /* Get the read out protection status */
  if (0)
  {
    while(1)
    {
      STM_EVAL_LEDToggle(LED5);
      Delay(10);
    }    
  }
  else
  {
    /*if (WaveRecStatus == 1)
    {
      WaveFileName = REC_WAVE_NAME;
    }
    else
    {
      WaveFileName = WAVE_NAME; 
    }*/
    /* Open the wave file to be played */
    if (f_open(&fileR, fname , FA_OPEN_ALWAYS | FA_READ) != FR_OK)
    {
      STM_EVAL_LEDOn(LED5);
      Command_index = 1;
    }
    else
    {    
      /* Read data(_MAX_SS byte) from the selected file */
      f_read (&fileR, buffer1, _MAX_SS, &BytesRead);
      
      WaveFileStatus = WavePlayer_WaveParsing(&wavelen);
      
      if (WaveFileStatus == Valid_WAVE_File)  /* the .WAV file is valid */
      {
        /* Set WaveDataLenght to the Speech wave length */
        WaveDataLength = WAVE_Format.DataSize;
      }
      else /* invalid wave file */
      {
        return -1;
      }
      /* Play the wave */
	  int cnt;
	  f_read(&fil,readBuf,sizeof(readBuf),&cnt);
      bytesLeft += cnt;
      readPtr = readBuf;
      WavePlayBack(WAVE_Format.SampleRate);
    }    
  }
}
/*move from waveplayer*/
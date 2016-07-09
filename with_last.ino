#include "Sodaq_wdt.h"

#include <UTFT_SPI.h>
#include <SD.h>
#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include "memorysaver.h"
#include <itoa.h>

#define RED_LED 13
#define GREEN_LED 8

#define SD_CS 4
const int SPI_CS = 5;

ArduCAM myCAM(OV5642, SPI_CS);
UTFT myGLCD(SPI_CS);

// datファイル名
#define WITH_DAT "WITH.DAT"

// Nミリ秒に一回撮影
#define SPAN_SPAN_MS 1000 * 15  // 15で実質20秒毎に撮影

int SPI_ERROR_COUNT = 1;
int SPI_RETRY_COUNT = 10;


void reset() {
}

void cam_test() {

  uint8_t temp;

  if (SPI_ERROR_COUNT >= SPI_RETRY_COUNT) {
    return;
  }

  // SPI_CS を初期化
  pinMode(SPI_CS, OUTPUT);
  SPI.begin();
  
  // カメラテスト
  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  temp = myCAM.read_reg(ARDUCHIP_TEST1);
  Serial.println(temp);
  if (temp != 0x55) {
    Serial.println("SPI interface Error!");
    myCAM.set_bit(ARDUCHIP_GPIO, GPIO_PWDN_MASK);
    Serial.print("SPI ERROR COUNT :");
    Serial.println(SPI_ERROR_COUNT);
    reset();
    int error_loop_count = 0;
    while (1) {
      digitalWrite(RED_LED, HIGH);
      sodaq_wdt_safe_delay(100);
      digitalWrite(RED_LED, LOW);
      sodaq_wdt_safe_delay(100);
      error_loop_count++;
      if (error_loop_count >= 20) {
        SPI_ERROR_COUNT = SPI_ERROR_COUNT + 1;
        cam_test();
        Serial.print("SPI ERROR COUNT 1 :");
        Serial.println(SPI_ERROR_COUNT);
        if (SPI_ERROR_COUNT >= SPI_RETRY_COUNT) {
          break;
        }
        break;
      }
    }
  }

  return;

}


void setup() {

  uint8_t vid, pid;

  // シリアル通信を開始
  Serial.begin(9600);
  Wire.begin();

  // LED初期化
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  // SDカードを初期化
  SD.begin(SD_CS);
  Serial.println("SD.begin 02 :)");

  // カメラをテストする
  cam_test();

  myCAM.write_reg(ARDUCHIP_FRAMES, 0x00);
  myCAM.set_bit(ARDUCHIP_GPIO, GPIO_RESET_MASK);
  myCAM.rdSensorReg8_8(OV5642_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg8_8(OV5642_CHIPID_LOW, &pid);
  if ((vid != 0x56) || (pid != 0x42)) {
    Serial.println("Can't find OV5642 module!");
  }
  else {
    Serial.println("OV5642 detected");
  }

  // JPEGに設定
  myCAM.set_format(JPEG);

  // カメラ初期化
  myCAM.InitCAM();
  myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);    //VSYNC is active HIGH

  // 画像サイズ指定
  myCAM.OV5642_set_JPEG_size(OV5642_2592x1944);

  myCAM.clear_fifo_flag();
  myCAM.write_reg(ARDUCHIP_FRAMES, 0x00);

  // 節電のため一旦カメラをスリープさせる
  Serial.println("Camera Sleep 01");
  myCAM.set_bit(ARDUCHIP_TIM, FIFO_PWRDN_MASK);
  myCAM.set_bit(ARDUCHIP_TIM, LOW_POWER_MODE);
  myCAM.set_bit(ARDUCHIP_GPIO, GPIO_PWDN_MASK);

  Serial.println("With Start! Have a good Trip :)");
  sodaq_wdt_flag = true;
}

void loop()
{

  Serial.println("Loop...");
  if (sodaq_wdt_flag) {

    Serial.println("sodaq_wdt_flag INSIDERS");
    // Wakeup!
    Serial.println("Wake uped! Good morning :)");
    sodaq_wdt_disable();
    // ここで処理を実行する
    // 撮影する
    snap();

    Serial.println("SNAP END :)");
    sodaq_wdt_flag = false;
    sodaq_wdt_reset();
    sodaq_wdt_enable(WDT_PERIOD_1X);
    Serial.println("SNAP END 02 :)");

  }
  Serial.println("sodaq_wdt_flag OUTSIDE");
  // スリープする
  systemSleep();
}


boolean now_snapping = false;
int buf_length = 48;

void error_led(int counter) {
  for (int i = 0; i < counter; i++) {
    digitalWrite(RED_LED, HIGH);
    sodaq_wdt_safe_delay(100);
    digitalWrite(RED_LED, LOW);
    sodaq_wdt_safe_delay(100);
  }
}

void snap() {

  uint8_t temp, temp_last;
  int total_time = 0;
  static int i = 0;
  File JPEG_FILE;
  byte buf[buf_length];

  if (now_snapping) {
    Serial.println("Sorry... Now Snapping...");
    return;
  }

  digitalWrite(RED_LED, HIGH);

  // ファイル名
  char jpeg_file_name[] = "with.dat";

  now_snapping = true;
  Serial.println("Snap Called...");

  // カメラをアクティブに
  myCAM.clear_bit(ARDUCHIP_TIM, FIFO_PWRDN_MASK);
  myCAM.clear_bit(ARDUCHIP_TIM, LOW_POWER_MODE);
  myCAM.clear_bit(ARDUCHIP_GPIO, GPIO_PWDN_MASK);
  delay(150);

  // fifo を初期化
  myCAM.flush_fifo();

  // 撮影フラグを初期化
  myCAM.clear_fifo_flag();

  // 撮影
  Serial.println("Start Capture");
  digitalWrite(GREEN_LED, HIGH);
  delay(300); // 光を取り込むために少し delay。（これをしないと、色が変色します）
  myCAM.start_capture();

  // 撮影が完了するまで待機
  while (!myCAM.get_bit(ARDUCHIP_TRIG , CAP_DONE_MASK)) {
  }
  digitalWrite(GREEN_LED, LOW);
  Serial.println("END Capture :)");


  uint32_t length = 0;

  // fifoサイズ（画像サイズ）を取得
  length = myCAM.read_fifo_length();
  Serial.print("file size : ");
  Serial.println(length);

  if (length >= 524287) {
    // 大きすぎるとエラー
    Serial.println("FILE SIZE ERROR 01 :(");
    myCAM.clear_fifo_flag();
    now_snapping = false;
    Serial.println("Camera Sleep 05 :(");
    myCAM.set_bit(ARDUCHIP_TIM, FIFO_PWRDN_MASK);
    myCAM.set_bit(ARDUCHIP_TIM, LOW_POWER_MODE);
    myCAM.set_bit(ARDUCHIP_GPIO, GPIO_PWDN_MASK);
    error_led(5);
    return;
  }

  if (length == 0) {
    // 0バイトも当然エラー
    Serial.println("FILE SIZE ERROR 02 :(");
    myCAM.clear_fifo_flag();
    now_snapping = false;
    Serial.println("Camera Sleep 06 :(");
    myCAM.set_bit(ARDUCHIP_TIM, FIFO_PWRDN_MASK);
    myCAM.set_bit(ARDUCHIP_TIM, LOW_POWER_MODE);
    myCAM.set_bit(ARDUCHIP_GPIO, GPIO_PWDN_MASK);
    error_led(5);
    return;
  }

  length = 0;
  // カメラをスリープ
  Serial.println("Camera Sleep 02 :)");
  myCAM.set_bit(ARDUCHIP_TIM, FIFO_PWRDN_MASK);
  myCAM.set_bit(ARDUCHIP_TIM, LOW_POWER_MODE);
  myCAM.set_bit(ARDUCHIP_GPIO, GPIO_PWDN_MASK);

  // SDに書き込んでいく
  Serial.println("SD Write start ;)");

  total_time = millis();

  // ファイルを開く
  JPEG_FILE = SD.open(jpeg_file_name, O_CREAT | O_APPEND | O_WRITE);
  if (!JPEG_FILE) {
    now_snapping = false;
    digitalWrite(RED_LED, LOW);
    Serial.println("Cant oepn file ;(");
    error_led(2);
    return;
  }


  i = 0;
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();

  // fifoから1バイト読込
  Serial.println("REED Byte ;)");
  temp = SPI.transfer(0x00);

  int total_write = 0;

  while ( (temp != 0xD9) | (temp_last != 0xFF)) {
    //    Serial.println("REED Byte Loop ;)");
    length++;
    temp_last = temp;
    temp = SPI.transfer(0x00);
    if (i < buf_length) {
      buf[i++] = temp;
    }
    else
    {
      //Write 256 bytes image data to file
      myCAM.CS_HIGH();
      JPEG_FILE.write(buf, buf_length);
      i = 0;
      buf[i++] = temp;
      myCAM.CS_LOW();
      myCAM.set_fifo_burst();
    }
    total_write++;
    if (total_write > 524288) {
      break;
    }
  }
  if (i > 0) {
    Serial.println("LAST WRITE!");
    myCAM.CS_HIGH();
    JPEG_FILE.write(buf, i);
  }

  // セパレータを書き込む
  JPEG_FILE.write("\nWITH_SEPARATOR\n", 16);

  Serial.println("JPEG_FILE.close");
  JPEG_FILE.close();

  total_time = millis() - total_time;
  Serial.print("Total time used:");
  Serial.print(total_time, DEC);
  Serial.println(" millisecond");
  Serial.println("clear_fifo_flag");


  now_snapping = false;
  digitalWrite(RED_LED, LOW);

  Serial.println("Snap END...");

}





// SLEEPモードに入る
void systemSleep() {

  Serial.println("systemSleep() Called");
  if (!sodaq_wdt_flag) {
    Serial.println("systemSleep() Called 01");
    USBDevice.detach();
    Serial.println("systemSleep() Called 02");

    // Set the sleep mode
//    SCB->SCR |= 1 << 2;
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    Serial.println("systemSleep() Called 03");
    __WFI();
    Serial.println("systemSleep() Called 04");
    // Power off LED after sleep
    sodaq_wdt_safe_delay(SPAN_SPAN_MS);
    Serial.println("systemSleep() Called 05");
  }
  Serial.println("systemSleep() Called 06");
}


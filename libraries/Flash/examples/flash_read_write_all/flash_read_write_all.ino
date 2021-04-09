
#include <Flash.h>


// uint32_t blockdata[FLASH_SIZE >> 2];


void setup()
{
	Serial.begin(115200);
  // memset((uint8_t *)(Flash.blockdata), 0, 512);
    delay(3000);
    printBlockData();
    delay(1000);
    writeBlockData();
    delay(1000);
    printBlockData();
}

void loop()
{
	Serial.println("loop ...");
	delay(1000);
}


void printBlockData(void)
{
    uint32_t data32;
    Serial.println("reading block data");
    Flash.readAll();
    Serial.println("print block data");
    for (uint16_t i = 0; i < (FLASH_SIZE >> 2); i++)
    {
        data32 = Flash.blockdata[i];
        Serial.print(data32);
        Serial.print("\t");
        if ((i % 8) == 7) Serial.println("");
    }

    Serial.println("");
    Serial.println("print finish!");
}

void writeBlockData(void)
{
  uint32_t data32;
  Serial.print("size of: ");
   Serial.println(sizeof(Flash.blockdata));
    for (uint16_t i = 0; i < (FLASH_SIZE >> 2); i++)
    {
      Serial.println(i);
        Flash.blockdata[i] = 0xffff;
    }
    Serial.println("print blockdata, should be 123456");
//     for (uint16_t i = 0; i < (FLASH_SIZE >> 2); i++)
//     {
//         data32 = Flash.blockdata[i];
//         Serial.print(data32);
//         Serial.print("\t");
//         if ((i % 8) == 7) Serial.println("");
//     }
//     Serial.println("print finish!! start to write");
//     Flash.writeAll();
//     Serial.println("finish write");
}

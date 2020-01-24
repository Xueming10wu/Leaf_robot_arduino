#include "UDP.h"


UDP::UDP()
{
    //lcd = new LiquidCrystal_I2C(0x27,20,4); 
    //lcd->init();                      // initialize the lcd 
    //lcd->backlight();

    setBufferSize(256);
    udata = new uint8_t [1];
    recieveStatus = 1;
}

UDP::~UDP()
{
    delete []udata;
    udata = NULL;
}

void UDP::setMessageValueSize(message& msg, int size)
{
    msg.valueSize = size;
    msg.value = new uint8_t[msg.valueSize];
}

void UDP::setStart(message& msg, uint8_t signal_)
{
    msg.start = signal_;
}

void UDP::setCrc(message& msg)
{
    int sum = 0;
    sum += msg.start;
    sum += msg.end;
    for(int i = 0 ; i < msg.valueSize ; i ++)
    {
        sum += msg.value[i];
    }
    msg.crc = (uint8_t)(sum& 0xff);
}

void UDP::setEnd(message& msg, uint8_t signal_)
{
    msg.end = signal_;
}

int UDP::getMessageValueSize(message& msg) const
{
    return msg.valueSize;
}

uint8_t UDP::getStart(message& msg) const
{
    return msg.start;
}

uint8_t UDP::getCrc(message& msg) const
{
    return msg.crc;
}

uint8_t UDP::getEnd(message& msg) const
{
    return msg.end;
}

void UDP::setBufferSize(int size)
{
    bufferSize = size;
    txBuffer = new uint8_t[bufferSize];
    rxBuffer = new uint8_t[bufferSize];
}

int UDP::getBufferSize() const
{
    return bufferSize;
}

int UDP::write(message& msg)
{
    //转换到缓存中
    msg2udp(msg);
    //写入操作
    return Serial.write(txBuffer, 3 + msg.valueSize);
}


bool UDP::read(message& msg)
{
    if(simpleRecieve(msg.start, msg.end, 3 + msg.valueSize))
    {
        udp2msg(msg);
        int tempcrc = msg.start + msg.end;
        uint8_t crc = 0;
        for (size_t i = 0; i < msg.valueSize; i++)
        {
            tempcrc += msg.value[i];
        }
        crc = (uint8_t)(tempcrc & 0xff);
        if (crc == msg.crc)
        {
            return true;
        }
    }
    //printBuffer(rxBuffer, 4 + msg.valueSize);
    
    //校验位出错
    return false;
}


void UDP::flushTxBuffer()
{
    txIndex = 0;
    for(int i = 0 ; i < bufferSize ; i ++)
    {
        txBuffer[i] = 0x00;
    }
}

void UDP::flushRxBuffer()
{
    rxIndex = 0;
    for(int i = 0 ; i < bufferSize ; i ++)
    {
        rxBuffer[i] = 0x00;
    }
}

void UDP::flush()
{
    txIndex = 0;
    rxIndex = 0;
    for(int i = 0 ; i < bufferSize ; i ++)
    {
        txBuffer[i] = 0x00;
        rxBuffer[i] = 0x00;
    }
}

/*
void UDP::printBuffer(uint8_t* buf, int size)
{
    for(int i = 0 ; i < size ; i ++)
    {
        if(buf[i] >= 32 && buf[i] <= 126)
        {
            cout << (char)buf[i] << " ";
        }
        else
        {
            cout << "0x" << hex <<(int)buf[i] << " ";
        }
    }
    cout << "\n" << dec;
}*/


void UDP::msg2udp(message& msg)
{
    flushTxBuffer();
    txBuffer[0] = msg.start;
    memcpy(txBuffer + 1, msg.value, msg.valueSize);
    txBuffer[ 2 + msg.valueSize] = msg.end;
    setCrc(msg);
    txBuffer[ 1 + msg.valueSize] = msg.crc;
    //printBuffer(txBuffer, 4 + msg.valueSize);
}

void UDP::udp2msg(message& msg)
{
    msg.start = rxBuffer[0];
    memcpy(msg.value, rxBuffer + 1, msg.valueSize);
    msg.crc = rxBuffer[ 1 + msg.valueSize];
    msg.end = rxBuffer[ 2 + msg.valueSize];
    flushRxBuffer();
}


int UDP::recieve(uint8_t startSignal, uint8_t endSignal, int size)
{
    flushRxBuffer();
    //cout << "startSignal " << (char)startSignal << ",  endSignal " << endSignal << endl;
    if(recieveStatus == 0)
    {
        //通讯状态良好，整体接收
        Serial.readBytes(rxBuffer, size);
        if(rxBuffer[0] == startSignal && rxBuffer[size - 1] == endSignal)
        {
            recieveStatus = 0;
        }
        else
        {
            recieveStatus = 1;
        }
    }
    else
    {
        //通讯状态不好，逐个接收
        while( rxIndex < size)
        {
            if(Serial.available() <= 0)
            {
                return -1;
            }
            Serial.readBytes(udata, 1);
            rxBuffer[rxIndex] = udata[0];
            rxIndex += 1;
            if (rxBuffer[0] != startSignal)
            {
                //cout << "no right start :" <<(char)startSignal <<  endl;
                flushRxBuffer();
            }
            if(rxIndex > size - 1)
            {
                if( rxBuffer[size - 1] != endSignal)
                {
                    //cout << "no right end :"  <<(char)endSignal << endl;
                    flushRxBuffer();
                }
                else
                {
                    recieveStatus = 0;
                    break;
                }
            }
        }
    }
    return 0;
    /*
    //lcd->print("Status ");
    if (recieveStatus == 1)
    {
        //lcd->print("bad ");
    }else
    {
        //lcd->print("good ");
    }
    */
    
}


bool UDP::simpleRecieve(uint8_t startSignal, uint8_t endSignal, int size)
{
    flushRxBuffer();
    if(Serial.available() >= size)
    {
        Serial.readBytes(rxBuffer, Serial.available());
        if(rxBuffer[0] == startSignal && rxBuffer[size - 1] == endSignal)
        {
            return true;
        }
    }
    return false;
}
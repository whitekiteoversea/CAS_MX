//////////////////////////////////////////////////////////////////////////////////	 
//
//
//�洢ͼƬ���ݣ�ͼƬ��СΪ64*32����
//
/////////////////////////////////////////////////////////////////////////////////

#ifndef __BMP_H
#define __BMP_H
unsigned char BMP1[] =
{
/*--  ������һ��ͼ��D:\�ҵ��ĵ�\Pictures\1star128X64.bmp  --*/
/*--  ����x�߶�=128x64  --*/
0xFF,0xFF,0xFF,0xFF,0xFF,0xCF,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,//һ��16X8λ,64��/256������
0x0F,0x0F,0x0F,0xFF,0x3F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,
0x0F,0xCF,0xEF,0x3F,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0x3F,0x3F,0x1F,0x1F,0x1F,0x1F,
0x1F,0x1F,0x1F,0x1F,0x3F,0x3F,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0x1F,0x0F,0x0F,0x0F,
0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0x7F,0x3F,0x3F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x3F,
0x3F,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,
0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x3F,0x3F,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0x07,0x7E,0xF0,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xE0,0xFF,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xE0,0xF0,0x38,0x1E,
0x07,0x01,0x00,0x00,0xFF,0xFF,0xFF,0x01,0x00,0x00,0x00,0x00,0x00,0xF8,0xFC,0xFC,
0xFE,0xFC,0xFC,0x00,0x00,0x00,0x00,0x00,0x03,0xFF,0xFF,0xFF,0xFE,0xFC,0xFE,0xFE,
0xFE,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFF,0xFF,
0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0xFC,0xFE,0xFE,0xFE,0xFE,0xFC,0x00,0x00,
0x00,0x00,0x00,0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0xFC,0xFE,
0xFE,0xFE,0xFE,0xFC,0xF8,0x00,0x00,0x00,0x00,0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x0F,0xFF,0xFC,0xE0,0x00,0x00,0x00,0x00,0x00,
0xFC,0x1F,0x00,0x00,0x00,0x80,0xE0,0xF0,0xF8,0x3E,0x1F,0x07,0x01,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xF8,0xF0,0xE0,0x80,0x00,0x00,0x03,0x07,0x0F,
0x1F,0x7F,0xFF,0xFF,0xFE,0xFE,0xFE,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,
0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0x7F,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0x30,0x60,0xC0,0x80,0x0F,0x7F,0xFF,0xF8,0xC0,0xC0,0xDF,
0xC3,0xF0,0xF8,0xFC,0x3F,0x1F,0x07,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xFC,0xF8,0xF0,
0xC0,0x80,0x00,0x01,0x03,0x07,0x0F,0x1F,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x83,0x83,0x83,0x83,0x83,0x83,0x00,0x00,
0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0xE0,0xE1,
0xE1,0xE1,0xE1,0xE0,0x00,0x00,0x00,0x00,0x08,0x1C,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0x38,0x38,0x38,0x79,0x7D,0xFF,0xF7,0xE3,0xE3,0xF7,0xFF,
0xFF,0xFF,0xB9,0x38,0x38,0x38,0x38,0x38,0x38,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
0x18,0x10,0x10,0x10,0xFF,0xFF,0xFF,0xFF,0x03,0x03,0x03,0x03,0x03,0x03,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,
0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0x18,0x0C,0x06,0xC3,0xF9,0xFF,0x7F,0x3F,0x0F,0x03,0x0F,
0xFF,0xC7,0x0F,0x1F,0x3E,0x3C,0x78,0xF0,0xE0,0xC0,0x80,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFC,0xF8,0xF0,0xE0,0xC0,0xC0,0xC1,0x81,
0x83,0x83,0xC1,0xC0,0xC0,0xC0,0xE0,0xF0,0xF8,0xFC,0xFF,0xEF,0xC7,0xCF,0xFF,0xFF,
0xFF,0xFF,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xFF,0xFF,0xFF,0xEF,0xC7,0xCF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xC0,0xC0,0xC0,0xC0,0xC0,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xC0,0xC0,
0xC0,0xC0,0xC0,0xF8,0xFF,0xC7,0xC7,0xFF,0xFF,0xC0,0xC0,0xC0,0xC0,0xC0,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xC0,0xC0,0xC0,0xC0,0xC0,0xF8,0xFF,0xCF,0xC7,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xC0,0xF0,0x3E,0x0F,0x07,0x01,0x00,0x00,0x00,0x00,0x00,
0x00,0x07,0x7E,0xE0,0x00,0x00,0x00,0x00,0x01,0x03,0x07,0x0F,0x0E,0x18,0x30,0x60,
0xC0,0x80,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x03,0x03,0xFF,0xFF,0xFF,0xFF,0x3F,
0x07,0x63,0x07,0x3F,0xFF,0xFF,0x03,0x9B,0x9B,0x93,0x27,0x7F,0x0F,0xE3,0xFB,0xF9,
0xFB,0xF3,0x07,0xFF,0x03,0x93,0xBB,0x1B,0xC3,0xEF,0xFF,0x3F,0x0F,0x63,0x47,0x1F,
0xFF,0xFB,0xFB,0x93,0x03,0xFB,0xFB,0x9F,0x07,0xF3,0xFB,0xF9,0xF3,0x07,0x1F,0xFF,
0x03,0x9B,0x3B,0x13,0xC7,0xFF,0xFF,0xFF,0xFF,0x03,0xFF,0xFF,0xFF,0xFF,0x03,0x93,
0x9B,0x9B,0xFB,0xFF,0xE7,0xC3,0x9B,0x9B,0x33,0x77,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xF9,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,
0xF8,0xF8,0xF8,0xFF,0xFF,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,
0xF8,0xF9,0xFB,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0xF8,0xF9,0xF9,0xFF,0xFD,0xFC,
0xFF,0xFF,0xFF,0xFC,0xFD,0xFF,0xFC,0xF9,0xF9,0xF9,0xFC,0xFF,0xFF,0xFC,0xF9,0xF9,
0xF9,0xFC,0xFE,0xFF,0xFC,0xFD,0xFF,0xFE,0xFC,0xFF,0xFD,0xFC,0xFE,0xFF,0xFF,0xFC,
0xFC,0xFF,0xFF,0xFC,0xFC,0xFF,0xFF,0xFF,0xFE,0xFC,0xF9,0xF9,0xFD,0xFC,0xFF,0xFF,
0xFC,0xFF,0xFF,0xFC,0xFD,0xFF,0xFF,0xFF,0xFF,0xFC,0xFF,0xFF,0xFF,0xFF,0xFC,0xF9,
0xF9,0xF9,0xF9,0xFF,0xFE,0xFD,0xF9,0xF9,0xFC,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,

};

#endif


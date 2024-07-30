from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
import framebuf


i2c=I2C(0,sda=Pin(0), scl=Pin(1), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)

while True:
   TH = bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00[P\x00\x00\x00\x00\x00\x03\xad\xec\x0f\x00\x00\x00\x00\rv\xb6\x1a\x80\x00\x00\x00\x16\xdb[m\x80\x00\x00\x00um\xb5\xb6\x80\x00\x00\x00[\xb5n\xdb\x00\x00\x00\x01\xadW\xb5m\x80\x00\x00\x01w\xed[\xb6\x80\x00\x00\x03\xa8\x1a\xec+\x00\x00\x00\x05@\x17P\x1d\x00\x00\x00\x0e\x00\r\xb0\x06\x80\x00\x00\x08\x00\n\xd0\x00\x00\x00\x00\x08\x00\x17`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00@\x00\x00\x00\x00\x00\x00\x1f\xa0\x00\x00\x00\x00\x00\x00\n\xd0\x00\x00\x00\x00\x00\x00\r`\x00\x00\x00\x00\x00\x00\x17\xb0\x00\x00\x00\x00\x00\x00\n\xd0\x00\x00\x00\x00\x00\x00\x1b`\x00\x00\x00\x00\x00\x00\r\xb0\x00\x00\x00\x00\x00\x00\x16\xa0\x00\x00\x00\x00\x00\x00\x0bp\x00\x00\x00\x00\x00\x00\x1d\xa0\x00\x00\x00\x00\x00\x00\n\xd0\x00\x00\x00\x00\x00\x00\x17p\x00\x00\x00\x00\x00\x00\x1a\xa0\x00\x00\x00\x00\x00\x00\r\xd0\x00\x00\x00\x00\x00\x00\x16\xb0\x00\x00\x00\x00\x00\x00\x1b`\x00\x00\x00\x00\x00\x00\r\xb0\x00\x00\x00\x00\x00\x00\x16\xd0\x00\x00\x00\x00\x00\x00\x1bp\x00\x00\x00\x00\x00\x00\r\xa8\x00\x00\x00\x00\x00\x00\x16\xb0\x00\x00\x00\x00\x00\x00\x1b`\x00\x00\x00\x00\x00\x00\x15\xb0\x00\x00\x00\x00\x00\x00\x1e\xd0\x00\x00\x00\x00\x00\x00\x13p\x00\x00\x00\x00\x00\x00\x1d\xa8\x00\x00\x00\x00\x00\x00\x16\xb0\x00\x00\x00\x00\x00\x00\x15\xd0\x00\x00\x00\x00\x00\x00\x1a\xb8\x00\x00\x00\x00\x00\x00\x17`\x00\x00\x00\x00\x00\x00\x1a\xd8\x00\x00\x00\x00\x00\x00\x17h\x00\x00\x00\x00\x00\x00\x1a\xb0\x00\x00\x00\x00\x00\x00\x1d\xd8\x00\x00\x00\x00\x00\x00+h\x00\x00\x00\x00\x00\x00\x16\xb0\x00\x00\x00\x00\x00\x00\x1b\xd8\x00\x00\x00\x00\x00\x00\x16`\x00\x00\x00\x00\x00\x00\x0b\xa0\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00')
   fb = framebuf.FrameBuffer(TH,64,64, framebuf.MONO_HLSB)
   oled.fill(0)
   for i in range(-64,128):
       oled.blit(fb,i,0)
       oled.show()
   LOGO = bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x800\xc0\xc7\xf8~0\xc3\x0c\x1f\x87\xf8\x0c\x00\x00\x01\x800\xc1\xcf\xec\x7f0\xc7\x1c\x1f\xc7\xf8\x0c\x00\x00\x01\x800\xc1\xc1\x8ec\x99\xc6\x1c\x18\xe6\x00\x0c\x0c&`\x180\xc3\xe0\x06a\x99\xe6\x1e\x18f\x00\x1f>\x7f\xf8~0\xc3a\x8ea\x99\xe66\x18f\x00\x0cws\x98d?\xc3a\xbca\x99\xee6\x1f\xc7\xf0\x0ccs\x98`?\xc61\xb8a\x8f<s\x1f\x87\xf0\x0ccs\x18~0\xc61\x98a\x8f<c\x19\x86\x00\x0ccs\x18\x1e0\xc7\xf1\x9ca\x8f<\x7f\x99\xc6\x00\x0ccs\x18\x060\xcf\xf9\x8cc\x8e8\xff\x98\xc6\x00\x0f\x7fs\x18~0\xcc\x19\x8e\x7f\x06\x18\xc1\x98\xe7\xf8\x07>s\x18|0\xcc\x1d\x86|\x06\x18\xc1\xd8g\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00')
   fb = framebuf.FrameBuffer(LOGO,128,64, framebuf.MONO_HLSB)
   oled.fill(0)
   for i in range(-128,128):
       oled.blit(fb,i,0)
       oled.show()
import pyb, time
import sensor, image, time, math, ustruct

led = pyb.LED(3)
usb = pyb.USB_VCP()
while (usb.isconnected()==False):
   led.on()
   time.sleep(150)
   led.off()
   time.sleep(100)
   led.on()
   time.sleep(150)
   led.off()
   time.sleep(600)
   
sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)   # Set frame size to QQVGA for april tags
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
# AprilTags Test Measurements

sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
clock = time.clock()
# Note! Unlike find_qrcodes the find_apriltags method does not need lens correction on the image to work.

# What's the difference between tag families? Well, for example, the TAG16H5 family is effectively
# a 4x4 square tag. So, this means it can be seen at a longer distance than a TAG36H11 tag which
# is a 6x6 square tag. However, the lower H value (H5 versus H11) means that the false positve
# rate for the 4x4 tag is much, much, much, higher than the 6x6 tag. So, unless you have a
# reason to use the other tags families just use TAG36H11 which is the default family.

# The AprilTags library outputs the pose information for tags. This is the x/y/z translation and
# x/y/z rotation. The x/y/z rotation is in radians and can be converted to degrees. As for
# translation the units are dimensionless and you must apply a conversion function.

# f_x is the x focal length of the camera. It should be equal to the lens focal length in mm
# divided by the x sensor size in mm times the number of pixels in the image.
# The below values are for the OV7725 camera with a 2.8 mm lens.

# f_y is the y focal length of the camera. It should be equal to the lens focal length in mm
# divided by the y sensor size in mm times the number of pixels in the image.
# The below values are for the OV7725 camera with a 2.8 mm lens.

# c_x is the image x center position in pixels.
# c_y is the image y center position in pixels.

f_x = (16 / 3.984) * 160 # find_apriltags defaults to 2.8, not 16 if this if not set
f_y = (16 / 2.952) * 120 # find_apriltags defaults to 2.8 not 16 if this if not set
c_x = 160 * 0.5 # find_apriltags defaults to this if not set (the image.w * 0.5)
c_y = 120 * 0.5 # find_apriltags defaults to this if not set (the image.h * 0.5)


def near(x, base=10):
    return base * round(x/base)

def x_cm(x):
    return (-.9887*x - .046)


def z_cm(x):
    return (-3.315*x - 7.777)

def find_tag():
    tags = 0
    clock.tick()
    img = sensor.snapshot()
    for tag in img.find_apriltags(families=image.TAG16H5, fx=f_x, fy=f_y, cx=c_x, cy=c_y): # defaults to TAG36H11
        tags = tags + 1
    return tags

def collect_data():
    z = []
    x = []
    i = 0
    while(i < 20):
        clock.tick()
        img = sensor.snapshot()
        for tag in img.find_apriltags(families=image.TAG16H5, fx=f_x, fy=f_y, cx=c_x, cy=c_y): # defaults to TAG36H11
            z.append(z_cm(round(tag.z_translation(), 3))) # this is highly variable, need to trim
            x.append(x_cm(tag.x_translation()))
            i = i + 1

    z.sort()
    x.sort()

    return x, z

def trust(x, z):
    if( (x[-1] - x[0]) < 5 and (z[-1] - z[0]) < 10000):
        return 1
    return 0

while(True):
    cmd = usb.recv(4, timeout=5000)
    if (cmd == b'snap'):
        img = sensor.snapshot().compress()
        usb.send(ustruct.pack("<L", img.size()))
        usb.send(img)
        
    if (cmd == b'getz'):
        led.on()
        time.sleep(150)
        led.off()
        x, z = collect_data()
        data = z[10]
        usb.send(ustruct.pack("<L", data))
    
    if (cmd == b'getx'):
        led.on()
        time.sleep(150)
        led.off()
        x, z = collect_data()
        data = x[10]
        usb.send(ustruct.pack("<L", data))
        
    if (cmd == b'find'):
        led.on()
        time.sleep(150)
        led.off()
        tags = find_tag() 
        usb.send(ustruct.pack("<L", tags))
    
    if (cmd == b'trst'):
        led.on()
        time.sleep(150)
        led.off()
        x, z = collect_data()
        trust = trust(x, z) 
        usb.send(ustruct.pack("<L", trust))
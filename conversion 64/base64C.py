import base64, codecs, PIL
from PIL import Image
from io import BytesIO
from resizeimage import resizeimage

buffered = BytesIO()

picture = PIL.Image.open('bloggif_5adc0c6a8e456.jpeg')
#cover = resizeimage.resize_cover(picture, [200, 100])
picture.save(buffered, format="JPEG")
img_str = base64.b64encode(buffered.getvalue())
img_str='"imagen":"%s"'%(img_str)
#display image
picture.show()

# print whether JPEG, PNG, etc.
#print(img_str)
print (img_str)

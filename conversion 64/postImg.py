import requests
import base64, codecs, PIL
from PIL import Image
from io import BytesIO

buffered = BytesIO()

picture = PIL.Image.open('index2.jpg')

picture.save(buffered, format="JPEG")
img_str = base64.b64encode(buffered.getvalue())


payload="{\"img\":\"%s\"}"%(img_str)
headers={'Accept':'*/*','content-type':'application/json'}

print (payload)


r = requests.post("http://www.cansats3kratos.me/images/", data=payload,headers=headers)
print (r.status_code)
print (r.headers)

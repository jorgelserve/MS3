import requests, sys
import base64, codecs, PIL
from PIL import Image
from io import BytesIO

buffered = BytesIO()
argument=sys.argv[1]
picture = PIL.Image.open(argument)

picture.save(buffered, format="JPEG")
img_str = base64.b64encode(buffered.getvalue())


payload="{\"img\":\"%s\"}"%(img_str)
headers={'Accept':'*/*','content-type':'application/json'}

print (payload)


r = requests.post("http://www.cansats3kratos.me/images/", data=payload,headers=headers)
print (r.status_code)
print (r.headers)

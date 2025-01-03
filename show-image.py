import urllib.request 
from PIL import Image 
  
urllib.request.urlretrieve('http://192.168.1.246:5000/pic', "/tmp/pic.jpeg") 
  
img = Image.open("/tmp/pic.jpeg") 
img.show()

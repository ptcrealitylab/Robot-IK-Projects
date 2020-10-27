
with open('log10220', 'rb') as f: 
	fileContent = f.read()
	# byte = f.read(1)
	# while byte != "":
	# 	byte = f.read(1)
	# 	# print(byte.decode('ascii'))
	# 	# print(byte.decode('utf8'))
	# 	# print(byte.decode('latin1'))
	# 	# print(byte.decode('utf16'))
	# 	# print(byte.decode('utf32'))
	# 	# lines += byte

try: 
	print(fileContent.decode('utf16'))
except:
	pass

try:
	print(fileContent.decode('ascii'))
except: pass

try: 
	print(fileContent.decode('latin1'))
except: pass

try:
	print(fileContent.decode('utf8'))
except: pass

try:
	print(fileContent.decode('utf32'))
except: pass
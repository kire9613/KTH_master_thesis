cur_path = os.path.dirname(__file__)
new_path = os.path.relpath('/resources/param/obstacles.yaml', cur_path)

f = open('obstacles.yaml', "r")
print(f.read()) 

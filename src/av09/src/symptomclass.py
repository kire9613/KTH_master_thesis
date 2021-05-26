
#Class for symptom representation
class Symptom_obj:
    def __init__(self,objlist,faultsize):
        self.faultsize=faultsize
        self.pos_f_list=[]
        self.name=objlist[0]
            
        i=0
        while i<self.faultsize:
            self.pos_f_list.append(0)
            i=i+1
        
        for k in objlist:
            try:
                self.pos_f_list[int(k)-1]=1
            except:
                continue
        
        
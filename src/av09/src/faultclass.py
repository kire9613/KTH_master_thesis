

class Fault_obj:
    def __init__(self,objlist,sizesymps):
        self.sizesymps=sizesymps
        self.P_symps=[]
        
        k=0
        i=0
        while i<sizesymps:
            self.P_symps.append(0)
            i=i+1
    
        while k<len(objlist):
            
            objlist[k]=objlist[k].split(':')

            if k>0:
                self.P_symps[int(objlist[k][0])-1]=float(objlist[k][1])
            k=k+1
        
        self.name=objlist[0][0] 
        self.probmodel= objlist[0][1]
        self.modelparam=objlist[0][2]  
        self.severity=objlist[0][3]
            
        
       
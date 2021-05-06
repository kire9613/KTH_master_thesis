

#import stuff
import csv
import os


#class RUL_model for some components
class Newfault:
    def __init__(self, name,gamma):
        self.name=name
        self.gamma=gamma

#Fault with potential propagating faults. These propagating faults are stored in propfault.newF_list
class Propfault:
    def __init__(self,name):
        self.newF_list=[]
        self.name=name

#Creates all Propsfaults from the .csv file. The RUL model simply lowers the average lifetime of the component given that the component/subsystem has a minor failure
# The model is not necessarily accurate to reality but created as a show of concept.
class RUL_model:
    def __init__(self):
        self.prop_fault_list=[]
        THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
        my_file = os.path.join(THIS_FOLDER, 'prognosticssheet.csv')
        k=0
        with open(my_file) as csvfile:
            prog_file= csv.reader(csvfile, delimiter=',')
            for row in prog_file:
            
                if k>=1:
                    i=1
                    propfault=Propfault(row[0])
                    while i <= len(row)-1:
                        
                        try:
                            propfault.newF_list.append(Newfault(row[i].split(':')[0],row[i].split(':')[1]))
                        except:
                            print('')
                        i=i+1
                    self.prop_fault_list.append(propfault)
                k=k+1

    #Given the diagnosis, this method returns all possible propagating faults.
    def possible_new(self,diag):
        
        for fault in self.prop_fault_list:
            
            if fault.name==diag[0]:
                return fault.newF_list
        return [Newfault('NoFault',None)]
import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
import os

#print(firebase_admin)


THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
cred=credentials.Certificate(os.path.join(THIS_FOLDER,"serviceAccountKey.json"))

firebase_admin.initialize_app(cred)
        
database=firestore.client()

data=database.collection('Vehicle').document('Diagnostic_data').get()

print(data.to_dict())

database.collection(u'VehiclesTest').document(u'Vehicle1').collection(u'diagnosis').document(u'diagnoses').set({'dia':'hej'},merge=True)
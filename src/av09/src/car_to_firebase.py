#! /usr/bin/env python

import pyrebase
"""
<!-- The core Firebase JS SDK is always required and must be listed first -->
<script src="https://www.gstatic.com/firebasejs/8.3.1/firebase-app.js"></script>

<!-- TODO: Add SDKs for Firebase products that you want to use
     https://firebase.google.com/docs/web/setup#available-libraries -->
<script src="https://www.gstatic.com/firebasejs/8.3.1/firebase-analytics.js"></script>

<script>
  // Your web app's Firebase configuration
  // For Firebase JS SDK v7.20.0 and later, measurementId is optional
  var firebaseConfig = {
    apiKey: "AIzaSyC0oAIOkG2k9YJhLe72ts5pN6jraN43nzI",
    authDomain: "pytest-6b7a8.firebaseapp.com",
    projectId: "pytest-6b7a8",
    storageBucket: "pytest-6b7a8.appspot.com",
    messagingSenderId: "388418484066",
    appId: "1:388418484066:web:3717527ad1638eb3b89e3b",
    measurementId: "G-NEY7MFW2YP"
  };
  // Initialize Firebase
  firebase.initializeApp(firebaseConfig);
  firebase.analytics();
</script>"""


firebaseConfig = {
    "apiKey": "AIzaSyC0oAIOkG2k9YJhLe72ts5pN6jraN43nzI",
    "authDomain": "pytest-6b7a8.firebaseapp.com",
    "projectId": "pytest-6b7a8",
    "storageBucket": "pytest-6b7a8.appspot.com",
    "messagingSenderId": "388418484066",
    "appId": "1:388418484066:web:3717527ad1638eb3b89e3b",
    "measurementId": "G-NEY7MFW2YP"
  }

  firebase=pyrebase.initialize_app(firebaseConfig)

  storage=firebase.storage()
  database=firebase.database()

  


from PyQt5 import uic
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.uic import *
from functools import partial
import sys 
import re
import os
import datetime

class Save_win(QDialog):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        uic.loadUi('Filename.ui',self)

        self.pushCheck.clicked.connect(self.show_funct)
        self.nameLine.editingFinished.connect(partial(self.noBorder,self.nameLine))
        self.pushSave.clicked.connect(self.set_name)
        self.pushCancel.clicked.connect(self.delete_file)

    def noBorder(self,objCall):
        objCall.setStyleSheet('')

    def check_fct(self):
        check_obj = [self.nameLine]
        check = True
        for object in check_obj:
            try:
                text = object.currentText()
                if text == '-':
                    object.setStyleSheet('border: 2px solid red')
                    check = False
            except:
                text = object.text()
                nWords = len(re.findall(r'\w+', text))
                if nWords < 2 or nWords > 3:
                    object.setStyleSheet('border: 2px solid red')
                    check = False
        return check
    
    def show_funct(self):
        if self.check_fct():
            # Full Name
            try:
                name,surname = self.nameLine.text().lower().split()
                full_name = name + '.' + surname
            except:
                name,conj,surname = self.nameLine.text().lower().split()
                full_name = name + '.' + conj + '.' + surname
            # Date
            year = str(datetime.datetime.now().year)
            month = str(datetime.datetime.now().strftime("%b"))
            day = str(datetime.datetime.now().day)
            hour = str(datetime.datetime.now().hour)
            minute = str(datetime.datetime.now().minute)
            second = str(datetime.datetime.now().second)

            filename = full_name + '_' + year + month + day + '_' + 'h' + hour + 'm' + minute + 's' + second + '.csv'
            self.resultLine.setText(filename)
            self.pushSave.setEnabled(True)

    def set_name(self):
        old_file = os.path.join('../../data/Tests/', "filetmp2rename.csv")
        new_file = os.path.join('../../data/Tests/', self.resultLine.text())
        os.rename(old_file, new_file)
        self.close()
    
    def delete_file(self):
        os.remove('../../data/Tests/filetmp2rename.csv')
        self.close()

'''
def main():
    app = QApplication(sys.argv)
    w = Save_win() 
    w.show()
    sys.exit(app.exec_())

main()
'''

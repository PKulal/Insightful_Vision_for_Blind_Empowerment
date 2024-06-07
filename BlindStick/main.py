import mysql.connector

mydb = mysql.connector.connect(host="localhost",user="root",password="",database="blindstick")
mycursor = mydb.cursor()

mycursor.execute("SELECT buttonstatus from buttonstatus")
myresult=mycursor.fetchall()
result=str(myresult)
#print(result)
result=result.replace('[(','')
result=result.replace(')]','')
result=result.replace(',','')


print(result)


if(result=="'1'"):
    import faceexpress

if(result=="'2'"):
    import objectdt

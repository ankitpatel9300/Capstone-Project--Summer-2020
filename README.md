# Capstone-Project--Summer-2020
#### Capstone Project work- EMBT
#### Group :- 5
    Group Members:- 
   - Ankitkumar Patel :mask:
   - Krutika Fanse    :mask:
   - Gurvindar Bhullar :mask:

**Similar product in market**


[![image](https://github.com/ankitpatel9300/Capstone-Project--Summer-2020/blob/master/Screenshot%20from%202020-06-28%2017-10-08.png)](https://www.amazon.ca/Flame-King-Smart-Propane-Scale/dp/B07NRC2W4C)


>   FreeRTOS Bug:- FreeRTOS V812 portmacro.h: missing binary operator before token “long”
Posted by rtel on October 15, 2014
That one is not so mysterious.  Just take the cast off the front of the 8, so it becomes:
#define configMAX_PRIORITIES ( 8 )

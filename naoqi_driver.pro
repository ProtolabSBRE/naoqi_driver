HEADERS += include/naoqi_driver/*.h \
           include/naoqi_driver/*.hpp \
           include/naoqi_driver/converter/*.h \
           include/naoqi_driver/converter/*.hpp \
           include/naoqi_driver/event/*.h \
           include/naoqi_driver/event/*.hpp \
           include/naoqi_driver/publisher/*.h \
           include/naoqi_driver/publisher/*.hpp \
           include/naoqi_driver/recorder/*.h \
           include/naoqi_driver/recorder/*.hpp \
           include/naoqi_driver/service/*.h \
           include/naoqi_driver/service/*.hpp \
           include/naoqi_driver/subscriber/*.h \
           include/naoqi_driver/subscriber/*.hpp

SOURCES = src/*.cpp \
          src/converters/*.cpp \
          src/helpers/*.cpp \
          src/publishers/*.cpp \
          src/services/*.cpp \
          src/tools/*.cpp \
          src/event/*.cpp \
          src/recorder/*.cpp \
          src/subscribers/*.cpp \

INCLUDEPATH += include/ \
               /opt/ros/indigo/include/  \
               ../tabasco_base/include/


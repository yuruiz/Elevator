SOURCE=$(shell find jSimPack simulator -name '*.java')
CLASSES=$(subst .java,.class,$(SOURCE))
SUBCLASSES=$(subst .java,\$$*.class,$(SOURCE))

.PHONY: all code clean

all: code

code: $(CLASSES) $(SUBCLASES)

clean:
#	-for file in $(SUBCLASSES) $(CLASSES); do if [ -d $$file ]; then rm $$file; fi; done
	-for file in $(SUBCLASSES) $(CLASSES); do if [ -e $$file ]; then rm -v $$file; fi; done

$(CLASSES): %.class: %.java
	javac -O -Xlint:all -cp $(PWD) $<

# ensure the next line is always the last line in this file.
# vi:noet

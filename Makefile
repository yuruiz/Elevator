SOURCE_DIR = code
DIRS = $(subst ./code/,,$(shell find ./code -mindepth 1 -type d))
PACKAGES = $(subst /,.,$(DIRS))

.PHONY: doc clean

doc:
	rm -rf doc
	javadoc -overview $(SOURCE_DIR)/overview.html -package \
	    -sourcepath $(SOURCE_DIR) \
	    -use -d $@ \
	    -doctitle "18-649 Elevator API" -windowtitle "18-649 API" \
	    $(PACKAGES)
	    
clean:
	rm -rf doc
	    
# vi:noet

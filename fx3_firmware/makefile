# Makefile for renaming files and applying patch

OLD_FOLDER_NAME = cyfxbulksrcsink
NEW_FOLDER_NAME = gpiftousb
OLD = cyfxbulksrcsink
NEW = cyfxgpiftousb

# List of files to patch
PATCHES = makefile readme.txt .project .cproject cyfxbulkdscr.c cyfxgpiftousb.c cyfxgpiftousb.h



# Default target
all: copy_and_rename patch add_files

# Copy the original SDK example and rename two files to fit the patches 
copy_and_rename:
	@echo "Copying ./$(OLD_FOLDER_NAME)/ to $(NEW_FOLDER_NAME)/"
	cp -r ./$(OLD_FOLDER_NAME)/ ./$(NEW_FOLDER_NAME)/
	@echo "Renaming $(OLD).c to $(NEW).c"
	mv ./$(NEW_FOLDER_NAME)/$(OLD).c ./$(NEW_FOLDER_NAME)/$(NEW).c
	@echo "Renaming $(OLD).h to $(NEW).h"
	mv ./$(NEW_FOLDER_NAME)/$(OLD).h ./$(NEW_FOLDER_NAME)/$(NEW).h
	@echo "done"
	@echo

# Patch the files
patch:
	@for file in $(PATCHES); do \
		echo "Applying patch to $$file"; \
		patch -e $(NEW_FOLDER_NAME)/$$file ./patches/$$file.patch; \
	done
	@echo "done"
	@echo

# Add files that were not in the original SDK example and thus don't need patches
add_files:
	@echo "\nCopying new files that did not exist in the SDK example"
	cp -r ./patches/new_files/. ./$(NEW_FOLDER_NAME)
	@echo "done"
	@echo
	@echo "The patched code is now available at ./$(NEW_FOLDER_NAME)"

# Delete the generated files
cleanup:
	@echo "\nCleaning up generated or patched files"
	rm -r ./$(NEW_FOLDER_NAME)/

.PHONY: all copy_and_rename patch add_files cleanup

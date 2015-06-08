
# Generates documentation for mainline
.PHONY: doc
doc:
	$(eval HASH := $(shell git rev-parse master))
	git show $(HASH):astar.nim > astar.nim
	nim doc astar.nim
	git add astar.html
	git commit -m "Generate docs from $(HASH)"


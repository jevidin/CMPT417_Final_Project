# CMPT417_Final_Project

## Run code:

### New arguments:
* --output set custom results csv filename (Default = "results")
* --idcbs run idcbs (Default = False)
* --repeat runs CBS/IDCBS for all agent numbers up to number specified in instance. e.g. if map instance has 30 agents then it will run for agent 1 and 2, then 1, 2, and 3, up to 1,2,3 ... 30. (Default = False)

```
python run_experiments.py --instance instances/berlin.txt --solver CBS --output results_berlin --repeat --batch --idcbs

```

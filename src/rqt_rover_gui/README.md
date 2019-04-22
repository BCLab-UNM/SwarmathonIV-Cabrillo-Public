# Using git subtree to manage rqt_rover_gui as a subproject

We have to use the version of `rqt_rover_gui` from BCLab-UNM/SwarmBaseCode-ROS in our final submission, but it's missing a few features that make development a lot easier for our team.

To manage a forked version of this subproject, we can split the `rqt_rover_gui` package into a subtree. This way it's easy to switch back to the exact version in the upstream repository before the submission deadline.

## From the man page:
`git subtree split`

Repeated splits of exactly the same history are guaranteed to be
identical (i.e. to produce the same commit ids). Because of this,
if you add new commits and then re-split, the new commits will be
attached as commits on top of the history you generated last time,
so git merge and friends will work as expected.

## Example uses:
### Initial subtree setup and split
```
git remote add -f upstream git@github.com:BCLab-UNM/SwarmBaseCode-ROS.git
git checkout -b staging-branch upstream/master

git subtree split -P src/rqt_rover_gui -b rqt-rover-gui
git push git@github.com:darrenchurchill/rqt_rover_gui.git rqt-rover-gui:master
```

#### Remove our current rqt_rover_gui directory
```
git rm -r src/rqt_rover_gui
git commit -a -m 'Prepare rqt_rover_gui to become a subtree.'
```

#### Add the subtree
```
git fetch origin
git checkout -b subtree-add origin/master
git subtree add -P src/rqt_rover_gui git@github.com:darrenchurchill/rqt_rover_gui.git master
```


### New changes upstream
#### Fetch the changes and re-split them
```
git fetch upstream
git checkout -b staging-branch upstream/master
git subtree split -P src/rqt_rover_gui -b rqt-rover-gui
```

#### Push the changes to the subtree remote repository
```
git push git@github.com:darrenchurchill/rqt_rover_gui.git rqt-rover-gui:master
```

#### Merge the new changes into the devel branch in the subtree repository
```
# in a local subtree repository
git checkout devel
git pull
git merge origin/master
git push
```

#### Pull the new changes back into the main repository
```
git checkout master
git pull
git subtree pull -P src/rqt_rover_gui git@github.com:darrenchurchill/rqt_rover_gui.git devel
```

### New changes in our repository
#### Re-split the subdirectory into a synthetic project history
```
git fetch origin
git checkout -b staging-branch origin/master
git subtree split -P src/rqt_rover_gui -b rqt-rover-gui
```

#### Merge and squash the synthetic history to update our subtree
```
git subtree merge -P src/rqt_rover_gui --squash rqt-rover-gui
```

#### Push the changes to the subtree remote repository's devel branch
```
git push git@github.com:darrenchurchill/rqt_rover_gui.git rqt-rover-gui:devel
```


### Resetting our repository to the sub project's master branch
```
git fetch origin
git checkout -b staging-branch origin/master
git subtree pull -P src/rqt_rover_gui --squash git@github.com:darrenchurchill/rqt_rover_gui.git master
```
Note: If you haven't worked with the subtree in your local repository, you might first need to pull the remote sub project's `devel` branch:
```
git subtree pull -P src/rqt_rover_gui --squash git@github.com:darrenchurchill/rqt_rover_gui.git devel
```

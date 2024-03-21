echo CLONING ASTROLABE
git submodule update --init --recursive
cd astrolabe
echo FIXING GIT BRANCH
git checkout remotes/origin/HEAD && git checkout master
git pull
echo BUILDING ASTROLABE
./gradlew build
cd ..
# Git Tutorial
https://www.runoob.com/git/git-tutorial.html

# Some Standard
Git常用分支  
1. master   
也被叫做Production分支，用于发布到生产环境的代码，命名为master  
master分支上的代码必须是稳定、可用的  
不允许在该分支上直接提交，该分支只能从其他分支合并  
该分支上所有的Commit都必须有Tag  
2. develop  
日常开发的主要分支,用于发布release和合并feature，命名为develop  
develop分支可以进行代码的提交，用于维护旧的代码  
新的功能不应该直接在develop分支修改，需要新建一个feature分支  
release分支是基于develop创建的  
3. feature  
用于开发新的功能，开发完成后合并到develop分支并进行下一次release，命名规则feat/some_feature  
feature分支基于develop分支创建  
feature分支开发完成后必须合并到develop分支，一般来说，此时feature分支已经完成了它的生命周期，应该删除，也可以保留  
多人合作开发时，应该创建各自的负责的feature分支，开发测试完成后合并入develop  
4. release
用于预发布版本，用于发布到生产环境前的测试和修正, 命名规则release/0.1.0  
release分支基于develop分支创建  
release分支创建后可以在该分支进行测试和修正bug(不允许再添加新功能)  
release分支一旦创建后不允许再从develop分支合并代码  
release完成测试后合并到master和develop分支，同时在master上打上Tag记住release版本号，完成后删除该release分支  
5. hotfix  
当master分支版本线上程序出现bug时，需要对线上版本进行修正，命名规则为hotfix/somebug  
hotfix分支基于master分支创建  
开发完成后将该分支合并到master和develop分支，同时给master分支打上Tag, 完成后删除该分支  
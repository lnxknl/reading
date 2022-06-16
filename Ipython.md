
    - 第五章 本章介绍一些较为“高级”的内容，详细如何通过多种方法加速你的代码，例如并行化计算、动态C编译。前面介绍对于IPython来说轻而易举就能实现多核心和多计算机的分布式任
务，后面介绍如何通过IPtython使用Python的超集编写代码以动态地进行C语言编译来大大提速。
    - 第六章 介绍如何定制自己及的IPython，创建新的“魔法命令”，并在IPython notebook中使用定制的功能。
   代码段将会像下面这样：

      print("Running script.")
      x = 12
      print("'x' is now equal to {0:d}.".format(x))


   所有的命令行输入或输出将会是下面的形式：

      In [1]: run script.py
      Running script.
      'x' is now equal to 12.
      In [2]: x
      Out[2]: 12



          - ⑤使用%run命令运行脚本

    - ⑥使用%timeit命令快速测量时间

    - ⑦使用%pdb命令快速debug

    - ⑧使用Pylab进行交互式计算

    - ⑨使用IPython Notebook
    - NumPy 高性能多维数组矢量计算

    - SciPy 高级i数值计算算法

    - Matplotlib 绘图及交互式可视化

    - Matplotlib-basemap Matplotlib绘图工具箱

    - Network X 处理图问题

    - Pandas 处理任意的表数据

    - Python Image Library（PIL），图片处理算法

    - PySide Qt的Python封包，用于图形用户接口（GUI）

    - PyQt 和PySide类似，但使用不同的用户协议

    - Cython 在Python中使用C语言代码
   这些命令中有些用来帮助你获得任意的Python函数和对象的心意。比如，你是否遇到过对怎样在派生类中使用super函数访问父类的方法？只需敲下super（%pinfo super 的快捷方式）

   你就会找到与super函数相关的所有信息。appending？或？？等等，所有的命令或变量你都能获得他们的信息。使用示例如下：

      In [1]: super?



         这些命令尤其是“魔法命令”是IPython的核心。在本书中我们将会使用数十个魔法命令。你可以通过魔法命令%lsmagic获得所有魔法命令的列表。


	    在任何交互会话中，你的所有输入和输出历史将会被保存在In和Out变量中，并被编号进行索引。_,__,___和_i,__i,___i变量保存着

   最后三个输出和输入对象。_n和_in变量返回第n个输出和输入历史命令。比如，让我们尝试一下

      In [4]: a = 12
      In [5]: a ** 2
      Out[5]: 144
      In [6]: print("The result is {0:d}.".format(_))
      The result is 144.



         在一个交互会话中你可以使用%timeit魔法命令快速测量代码运行时间。这可以让你评估单挑命令的运行时长。相同的命令会在一个循环中多次执行，多次运行时长的平均值作为该命令的最终
评估时长。-n选项控制命令在单次循环中执行的次数，-r选项控制执行循环的次数。例如
      In[1]: %timeit [x*x for x in range(100000)]
      10 loops, best of 3: 26.1 ms per loop



         IPython带有一个强大的调试器。无论何时控制台抛出了一个异常，你都可以使用%debug魔法命令在异常点启动调试器。接着你就能调试模式下访问所有的本地变量和整个栈回溯。使u和d向上
向下访问栈，使用q推出调试器。在调试器中输入？可以查看所有的可用命令列表。
   你也可以使用%pdb魔法命令去激活IPython调试器，这样，每当异常抛出时，调试器就会自动运行。



      %pylab魔法命令可以使Numpy和matplotlib中的科学及算功能生效，这些功能被称为基于向量和矩阵的高效操作，交互式可视化特性。这使得在控制台进行交互式计算和动态绘图称为可能。如
：
      In [1]: %pylab
      Welcome to pylab, a matplotlib-based Python environment [backend: TkAgg].
      For more information, type 'help(pylab)'.
      In [2]: x = linspace(-10., 10., 1000)
      In [3]: plot(x, sin(x))


      打开http://127.0.0.1:8888/创建一个新的Notebook
   你可以在输入框内编写多行代码。这里是一些常用的快捷键：

    - ①Enter 在输入框内创建新的一行且不执行这个输入框中的代码；

    - ②Shift+Enter 执行这个框内的代码并转到下一个框内；

    - ③Alt+Enter 执行框内代码，并在之后追加一个空的输入框

    - ④Ctrl+Enter 当你不想保存输出时，进行快速的实例实验；

    - ⑤Ctrl+M 接着K 显示所有的键盘快捷键。

   代码运行在2GHz的单核处理器上。每一个时钟周期理论上能够处理四个浮点操作，也就是每秒能进行80亿次操作。在我们的算法中，每一次迭代包含5次数学运算和一次对比操作，所以共有50
0W次操作（只计算数学运算）。理论上的最佳性能应该是6.25ms。这意味着我们的算法比理论值糟糕2600倍！
   当然，这是一次非常幼稚的评测，且理论值在实际中很难达到。但是2600倍的差距还是非常蛋疼糟糕的。我们能做的更好吗？我们将会在下一章节进行讲述。



         def closest(position, positions):
          x0, y0 = position
          dbest, ibest = None, None
          for i, (x, y) in enumerate(positions):
              # squared Euclidean distance from every position to the position of interest
              d = (x - x0) ** 2 + (y - y0) ** 2
              if dbest is None or d < dbest:
                  dbest, ibest = d, i
          return ibest


   在这里我们遍历寻找了每一个地点。变量i保存当前位置(x,y)的索引。我们感兴趣的地点是position(x0,y0)。在首次迭代中，当前位置一直保存最佳目标，在接下来的迭代中如果有比当前坐
标更与选找到的最近地点之间的平方距离保存在dbest。我们可以使用欧几里得距离公式来计算这个距离：$D=(X-x_0)^2+(Y-y_0)^2$趣的地点

   这是一个基础的标准算法。让我们在一个大数据集上来评估一下这个算法的性能。我们首先像下面这样随机产生1000W的随机地点：

      In[1]:import random
      In[2]:positions = [(random.random(),random.random()) for _ in xrange(10000000)]


   我们定义了一个有1000W个0到1之间的随机数字对组成的数组——position。现在，让我们使用下面的命令来评测一下：

      In[3]:%timeit closest((.5,.5),position)
      1 loops, best of 3: 16.4 s per loop


   这个算法处理1000W个地点花费了16.4秒。让我们来看一下这是否接近一个CPU的理论最大值。


      他们之间的不同是NumPy中的循环是用C代替Python进行实现的，这样就节省了循环中Python代码解释的时间。说实话，Python作为一种可交互的动态类型语言，其每一次的迭代操作包含着大
量的低级操作（类型检查等等）。这些操作花费的时间通常可以忽略不计，但是当他们重复几十万次的时候就会对性能造成很大的影响。

   这些就是为什么NumPy的矢量运算比Python自身的循环操作高效的多的主要原因。由于NumPy数组操作中相同的计算操作应用到了多个元素上面，从而参照实现了单指令、多数据（SIMD）计算
范式。我们将在先前示例的帮助下来证明这一点。


   我们可以看到NumPy允许以一种十分简单的语法来进行响亮操作。使用数组进行计算是一种非常特殊的编程方式，掌握她需要一些时间。在大多数语言中这是完全不同于编程的标准顺序方式，
但是，正如下面所示，在Python中这是相当高效的：
      In [7]: %timeit exec(In[6])
      1 loops, best of 3: 508 ms per loop


         为了方便起见，我们使用%bookmark citiesdata data命令为新创建的文件夹创建一个alias。现在我们就使用Pandas加载已经被解压的CSV文件。Pandas的read_csv函数能够打开任意的CSV文
件，就像下面这样：
      In[9]:import pandas as pd
      In[10]:filename = 'data/worldcitiespop.txt'
      In[11]:data = pd.read_csv(filename)


   现在让我们来探索一下这个新创建的data对象：

      In[12]：type[data]
      In[13]:pandas.core.frame.DataFrame


            In[13]:data.shape,data.keys()
      Out[13]:((3173958, 7),
      Index([Country, City, AccentCity, Region, Population, Latitude,
      Longitude], dtype=object))

            In [17]: data[data.AccentCity=='New York']

	          In [28]: population.describe()
      count 47980.000000
      mean 47719.570634
      std 302888.715626
      min 7.000000
      25% 3732.000000
      50% 10779.000000
      75% 27990.500000
      max 31480498.000000


            In [1]: import pandas as pd
      In [2]: cd citiesdata
      In [5]: filename = 'worldcitiespop.txt'
      In [4]: data = pd.read_csv(filename)
      In [5]: plot(data.Longitude, data.Latitude, ',')

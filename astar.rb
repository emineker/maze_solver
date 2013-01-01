# encoding: utf-8

# A* algoritmasını kullanarak labirent çözümü üret
# üretilen çözümleri adım adım görüntüle

require 'theseus'
# require 'theseus/orthogonal_maze'
require 'theseus/solvers/base'
require 'fileutils'

# An implementation of the A* search algorithm. Although this can be used to
# search "perfect" mazes (those without loops), the recursive backtracker is
# more efficient in that case.
#
# The A* algorithm really shines, though, with multiply-connected mazes
# (those with non-zero braid values, or some symmetrical mazes). In this case,
# it is guaranteed to return the shortest path through the maze between the
# two points.
class Astar < Theseus::Solvers::Base

  # This is the data structure used by the Astar solver to keep track of the
  # current cost of each examined cell and its associated history (path back
  # to the start).
  #
  # Although you will rarely need to use this class, it is documented because
  # applications that wish to visualize the A* algorithm can use the open set
  # of Node instances to draw paths through the maze as the algorithm runs.
  class Node
    # düğüm nesnelerinin karşılaştırılabilmesi bu nesne gerekli
    include Comparable

    # The point in the maze associated with this node.
    attr_accessor :point

    # Whether the node is on the primary plane (+false+) or the under plane (+true+)
    attr_accessor :under

    # The path cost of this node (the distance from the start to this cell,
    # through the maze)
    attr_accessor :path_cost

    # The (optimistic) estimate for how much further the exit is from this node.
    attr_accessor :estimate

    # The total cost associated with this node (path_cost + estimate)
    attr_accessor :cost

    # The next node in the linked list for the set that this node belongs to.
    attr_accessor :next

    # The array of points leading from the starting point, to this node.
    attr_reader :history

    def initialize(point, under, path_cost, estimate, history) #:nodoc:
      @point, @under, @path_cost, @estimate = point, under, path_cost, estimate
      @history = history
      @cost = path_cost + estimate
    end

    def <=>(node)
      cost <=> node.cost
    end
  end

  # The open set. This is a linked list of Node instances, used by the A*
  # algorithm to determine which nodes remain to be considered. It is always
  # in sorted order, with the most likely candidate at the head of the list.
  attr_reader :open

  def initialize(maze, a=maze.start, b=maze.finish)
    super
    @open = Node.new(@a, false, 0, estimate(@a), [])
    @visits = Array.new(@maze.height) { Array.new(@maze.width, 0) }
  end

  def current_solution
    @open.history + [@open.point]
  end

  def step
    return false unless @open

    current = @open

    if current.point == @b
      @open = nil
      @solution = current.history + [@b]
    else
      @open = @open.next

      @visits[current.point[1]][current.point[0]] |= current.under ? 2 : 1

      cell = @maze[current.point[0], current.point[1]]

      directions = @maze.potential_exits_at(current.point[0], current.point[1])
      directions.each do |dir|
        try = current.under ? (dir << Theseus::Maze::UNDER_SHIFT) : dir
        if cell & try != 0
          point = move(current.point, dir)
          next unless @maze.valid?(point[0], point[1])
          under = ((@maze[point[0], point[1]] >> Theseus::Maze::UNDER_SHIFT) & @maze.opposite(dir) != 0)
          add_node(point, under, current.path_cost+1, current.history + [current.point])
        end
      end
    end

    return current
  end

  private

  def estimate(pt)
    Math.sqrt((@b[0] - pt[0])**2 + (@b[1] - pt[1])**2)
  end

  def add_node(pt, under, path_cost, history)
    return if @visits[pt[1]][pt[0]] & (under ? 2 : 1) != 0

    node = Node.new(pt, under, path_cost, estimate(pt), history)

    if @open
      p, n = nil, @open

      while n && n < node
        p = n
        n = n.next
      end

      if p.nil?
        node.next = @open
        @open = node
      else
        node.next = n
        p.next = node
      end

      # remove duplicates
      while node.next && node.next.point == node.point
        node.next = node.next.next
      end
    else
      @open = node
    end
  end

  def move(pt, direction)
    [pt[0] + @maze.dx(direction), pt[1] + @maze.dy(direction)]
  end
end

STEPDIR = "steps/"

# adımlama dizini yoksa oluştur
Dir.mkdir(STEPDIR) unless File.exist?(STEPDIR)

# dizindeki eski resimleri temizle
FileUtils.rm_rf("#{STEPDIR}.")

# herhangi bir çoklu bağlantılı labirent oluştur
maze = Theseus::SigmaMaze.new(width: 10, height: 10, braid: 10)
puts "labirent üretiliyor"
maze.generate!

# A* arama algoritması için yeni bir çözüm nesnesi oluştur
# solver = Astar.new(maze, maze.start, maze.finish)
solver = Astar.new(maze)

puts "lebirent çözülüyor..."

step = 0

# daha önce kullanılmış ve artık bu yoldan biryere ulaşılamayacağı
# anlaşıldığında bu alanları gri ile renklendir
# eskimiş yollar
stale_paths = maze.new_path(color: 0x9f9f9fff)

while solver.step
  # devamı olan yolları "açık küme" şeklinde tanımla
  # ve bu yolları yeşil ile renklendir
  open_set = maze.new_path(color: 0x009600DD)

  # daha önce geçilen tüm yolları tanımla
  # ve bu yolları kırmızı ile renklendir
  histories = maze.new_path(color: 0xFF0000FF)

  # "en iyi" yol algoritması düşüncesinde en umut verici yolları tanımla
  # ve bu yolları pembe ile renklendir
  best = maze.new_path(color: 0xffaaaaff)

  # açık kümedeki ilk düğüm ile başla
  n = solver.open

  while n
    # açık kümeye yolun kendisini ekle
    open_set.set(n.point)

    # düğümü geçmiş kayıtlara yani kendi bağlantılarına ekle
    prev = maze.entrance
    n.history.each do |pt|
      how = histories.link(prev, pt)
      histories.set(pt, how)
      prev = pt
    end
    how = histories.link(prev, n.point)
    histories.set(n.point, how)
    n = n.next
  end

  if solver.open
    prev = maze.entrance
    solver.open.history.each do |pt|
      how = best.link(prev, pt)
      best.set(pt, how)
      prev = pt
    end
    best.link(prev, solver.open.point)
  elsif solver.solved?
    prev = maze.entrance
    solver.solution.each do |pt|
      how = best.link(prev, pt)
      best.set(pt, how)
      prev = pt
    end
    best.link(prev, maze.exit)
  end

  # geçilmiş yollara yeni geçilenleri ekle
  stale_paths.add_path(histories)

  # # süreç boyunca arka planda çalışan animasyon üretici için
  # fork do
  #   File.open("#{STEPDIR}step-%03d.png" % step, "w" ) do |f|
  #     f.write(maze.to(:png, cell_size: 30, background: 0x2f222222, paths: [best, open_set, histories, stale_paths]))
  #   end
  # end

  puts "%d. adım" % step
  step += 1
end

puts "#{step} adımda tamamlandı"

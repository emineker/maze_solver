# encoding: utf-8

require 'theseus/orthogonal_maze'
require 'theseus/solvers/base'
require 'fileutils'

# bu nesne, her bir hücrenin kontrolü sırasında ilişki geçmişini kontrol eder
# ve hesaplanmış maliyetler doğrultusunda kontrol edeceği bir sonraki noktayı seçer
#
# nadiren bu sınıfı kullanmamız gerekmesine rağmen,
# eğer çözüm algoritması görselleştirmek istenirse
# açık küme bağlantıları kullanılarak gerçekleştirilebilir
class Node
  # düğüm nesnelerinin karşılaştırılabilmesi için kullan
  include Comparable

  # labirentin bu düğüm ile ilgili olan noktası
  attr_accessor :point

  # düğüm birincil düzlemde ise yanlış, alt düzlemde ise doğru olsun
  attr_accessor :under

  # bu düğümün yol maliyeti: başlangıçtan itibaren bu düğüme olan mesafe
  # (labirent üzerinden yapılan hesap)
  attr_accessor :path_cost

  # çıkış bu düğümden tahminen (iyimser bir tahminle) ne kadar ilerde
  attr_accessor :estimate

  # bu düğüm ile ilişkili toplam maliyet (path_cost + tahmini)
  attr_accessor :cost

  # bağlantılı listesinde bu düğüme ait olan bir sonraki düğüm
  attr_accessor :next

  # başlangıç noktasından, bu düğüme giden noktaları dizisi
  attr_reader :history

  def initialize(point, under, path_cost, estimate, history)
    @point, @under, @path_cost, @estimate, @history = point, under, path_cost, estimate, history
    @cost = path_cost + estimate
  end

  def <=>(node)
    cost <=> node.cost
  end
end


class Solver < Theseus::Solvers::Base
  # A* algoritması tarafından kullanılan düğüm örnekleri
  # ve bağlantı listelerinin dikkate alınması gerekmektedir
  # bu liste her zaman en olasıdan başlayarak sıralanır
  attr_reader :open

  def initialize(maze)
    super
    @a, @b = maze.start, maze.finish
    @open = Node.new(@a, false, 0, estimate(@a), [])
    @visits = Array.new(@maze.height) { Array.new(@maze.width, 0) }
  end

  def step
    return unless @open

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
        try = current.under ? (dir << UNDER_SHIFT) : dir
        if cell & try != 0
          point = [current.point[0] + @maze.dx(dir), current.point[1] + @maze.dy(dir)]
          next unless @maze.valid?(point[0], point[1])
          under = ((@maze[point[0], point[1]] >> UNDER_SHIFT) & @maze.opposite(dir) != 0)
          add_node(point, under, current.path_cost+1, current.history + [current.point])
        end
      end
    end
  end

  private

  # tahmini yol hesabı
  def estimate(point)
    Math.sqrt((@b[0] - point[0])**2 + (@b[1] - point[1])**2)
  end

  def add_node(point, under, path_cost, history)
    return if @visits[point[1]][point[0]] & (under ? 2 : 1) != 0

    node = Node.new(point, under, path_cost, estimate(point), history)

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

      # tekrar edilenleri kaldır
      while node.next && node.next.point == node.point
        node.next = node.next.next
      end
    else
      @open = node
    end
  end

end

# adımların resimlerinin tutulduğu dizin
STEPDIR = "steps/"

# bit maskesi boyutu
UNDER_SHIFT = 8

# çözüm adımları
step = 0

# adımlama dizini yoksa oluştur
Dir.mkdir(STEPDIR) unless File.exist?(STEPDIR)

# dizindeki eski resimleri temizle
FileUtils.rm_rf("#{STEPDIR}.")

# theseus paketi ile çoklu bağlantılı labirent oluştur
maze = Theseus::OrthogonalMaze.new(width: 10, height: 10)
maze.generate!

puts maze
print "\nDevam etmek için bir tuşa basın..."
STDIN.gets.chomp


# bir çözüm nesnesi oluştur
solver = Solver.new(maze)

# eskimiş yollar: daha önce kullanılmış ve artık bu yoldan biryere
# ulaşılamayacağı anlaşıldığında gri ile renklendir
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

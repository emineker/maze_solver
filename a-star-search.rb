# encoding: utf-8

# A* algoritmasını kullanarak labirent çözümü üret
# üretilen çözümleri adım adım görüntüle
# sudo gem install theseus

require 'theseus'
require 'theseus/solvers/base'
require 'fileutils'
require './astar'


step_dir = "steps/"

# adımlama dizini yoksa oluştur
Dir.mkdir(step_dir) unless File.exist?(step_dir)

# dizindeki eski resimleri temizle
FileUtils.rm_rf("#{step_dir}.")

# herhangi bir çoklu bağlantılı labirent oluştur
maze = Theseus::OrthogonalMaze.new(width: 10, height: 10, braid: 10)
puts "labirent üretiliyor"
maze.generate!

# A* arama algoritması için yeni bir çözüm nesnesi oluştur
solver = Astar.new(maze, maze.start, maze.finish)

puts "lebirent çözülüyor..."

step = 0
renderings = 0

# daha önce kullanılmış ve artık bu yoldan biryere ulaşılamayacağı
# anlaşıldığında bu alanları gri ile renklendir
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

  # süreç boyunca arka planda çalışan animasyon üretici için
  # 6 karede bekleme süresi koy

  # while renderings > 6
  #   Process.wait
  #   renderings -= 1
  # end

  # renderings += 1

  # fork do
  #   File.open("#{step_dir}step-%03d.png" % step, "w" ) do |f|
  #     f.write(maze.to(:png, cell_size: 30, background: 0x2f222222, paths: [best, open_set, histories, stale_paths]))
  #   end
  # end

  puts "%d. adım" % step
  step += 1
end

# while renderings > 0
#   Process.wait
#   renderings -= 1
# end

puts "#{step} adımda tamamlandı"

# exec "montage -label '%f' stages/*.png -tile 5x -geometry 200x200\>+5+5 foo.png"

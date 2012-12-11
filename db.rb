
# Veri tabanı bağlantısı
dbconfig = YAML::load(File.open('database.yml'))

ActiveRecord::Base.establish_connection(dbconfig)

# logger
# ActiveRecord::Base.logger = Logger.new(STDERR)
ActiveRecord::Base.logger = Logger.new("database.log")


DB_TABLES = %w(knowledge_bases labyrinths)


def db_create
  ActiveRecord::Schema.define do
    # bilgi tabanı yoksa oluştur
    unless ActiveRecord::Schema.table_exists? :knowledge_bases
      create_table :knowledge_bases do |t|
        t.string :title
        t.string :performer
      end
    end

    # labirent bilgi tablosu yoksa oluştur
    unless ActiveRecord::Schema.table_exists? :labyrinths
      create_table :labyrinths do |t|
        t.string :coordinate
        t.string :performer
      end
    end
  end
end

def db_clear
  DB_TABLES.each do |table|
    ActiveRecord::Schema.drop_table(table)
  end
end


# models

class KnowledgeBase < ActiveRecord::Base
end

class Labyrinth < ActiveRecord::Base
end

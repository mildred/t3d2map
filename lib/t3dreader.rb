
module T3D

  class Section
  
    def self.read_section(io, line = nil)
      line = io.readline if line.nil?
      if line =~ /^\s*Begin (\S*)/
        name = $1
        args = Hash[line.scan(/(\S*)=(\S*)/)]
        case name
          when "PolyList" then
            return PolyList.read(io, args)
          when "Polygon" then
            return Polygon.read(io, args)
          else
            p line
            warn "Unknown object #{name}"
        end
      else
        p line
      end
    end
  
  end

  class PolyList < Section
    attr_reader :args
    attr_reader :polygons
    
    def initialize(args)
      @args = args
      @polygons = []
    end
  
    def self.read(io, args)
      obj = PolyList.new(args)
      while true
        l = io.readline
        return obj if l =~ /^\s*End PolyList/
        if l =~ /\s*Begin Polygon\s/
          obj.polygons << self.read_section(io, l)
        else
          p l
        end
      end
      nil
    end

  end

  class Polygon < Section
    attr_accessor :args
    attr_accessor :vertices
    attr_accessor :origin
    attr_accessor :normal
    attr_accessor :uvec
    attr_accessor :vvec
    attr_accessor :vvec
    
    def initialize(args)
      @args = args
      @vertices = []
    end
  
    def self.read(io, args)
      obj = Polygon.new(args)
      while true
        l = io.readline
        return obj if l =~ /^\s*End Polygon/
        if l=~ /^\s*(\S+)\s+([\S^,]+),([\S^,]+),([\S^,]+)/
          vector = [$2.to_f, $3.to_f, $4.to_f]
          case $1
            when "Vertex"   then obj.vertices << vector
            when "Origin"   then obj.origin   =  vector
            when "Normal"   then obj.normal   =  vector
            when "TextureU" then obj.uvec     =  vector
            when "TextureV" then obj.vvec     =  vector
            else
              p line
          end
        else
          p l
        end
      end
      nil
    end

  end

end


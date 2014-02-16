#!/usr/bin/env ruby

libdir = File.expand_path(File.join(File.dirname(__FILE__), 'lib'))
$:.unshift libdir

require 't3dreader'
require 'mapwriter'

T3D::Section.read_section($stdin)



with open('simulator_files.csv', 'r') as in_file:
  with open('simulator_labels.csv', 'w') as out_file:
    for path in in_file:
      color = 'None'
      if path.find('_0.') >= 0:
        color = 'Red'
      elif path.find('_1.') >= 0:
        color = 'Yellow'
      elif path.find('_2.') >= 0:
        color = 'Green'
      out_file.write(path.strip())
      out_file.write(',')
      out_file.write(color)
      out_file.write("\n")    

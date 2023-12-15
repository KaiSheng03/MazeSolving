#include "Cell.h"

Cell::Cell(){
  visited = false;
  useless = false;
}

Cell::Cell(int r, int c){
  row = r;
  column = c;
  visited = false;
  useless = false;
}

int Cell::getRow() const{
  return row;
}

int Cell::getColumn() const{
  return column;
}

void Cell::setCoordinate(int r, int c){
  row = r;
  column = c;
}

void Cell::setCaller(int r, int c){
  callerRow = r;
  callerColumn = c;
}

Cell Cell::getCaller() const{
  return Cell(callerRow, callerColumn);
}

Cell Cell::operator-(Cell& cell){
  return Cell(row-cell.row, column-cell.column);
}

bool Cell::operator==(const Cell& cell){
  if(row==cell.row && column==cell.column){
    return true;
  }
  else{
    return false;
  }
}

void Cell::markVisited(){
  visited = true;
}

void Cell::markUseless(){
  useless = true;
}

bool Cell::getVisited() const{
  return visited;
}

bool Cell::getUseless() const{
  return useless;
}

void Cell::printCoordinate() const{
  Serial.print("Row: ");
  Serial.print(row);
  Serial.print(" Column: ");
  Serial.println(column);
}
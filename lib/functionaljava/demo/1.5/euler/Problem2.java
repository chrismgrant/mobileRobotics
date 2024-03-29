package euler;

import fj.F2;
import fj.data.Stream;
import static fj.data.Stream.cons;
import static fj.function.Integers.even;
import static fj.function.Integers.sum;
import static fj.Ord.intOrd;

import static java.lang.System.out;

/**
 * Find the sum of all the even-valued terms in the Fibonacci sequence which do not exceed four million.
 */
public class Problem2 {
  public static void main(final String[] args) {
    final Stream<Integer> fibs = new F2<Integer, Integer, Stream<Integer>>() {
      public Stream<Integer> f(final Integer a, final Integer b) {
        return cons(a, curry().f(b).lazy().f(a + b));
      }
    }.f(1, 2);
    out.println(sum(fibs.filter(even).takeWhile(intOrd.isLessThan(4000001)).toList()));
  }
}

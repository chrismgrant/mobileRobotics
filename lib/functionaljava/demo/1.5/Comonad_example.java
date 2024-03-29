import fj.F;
import fj.P;
import static fj.data.List.asString;
import static fj.data.List.fromString;
import fj.data.Stream;
import static fj.data.Stream.join;
import static fj.data.Stream.single;
import fj.data.Zipper;
import static fj.data.Zipper.fromStream;

/**
 * Example of using a Zipper comonad to get all the permutations of a String.
 */

public class Comonad_example {
  public static void main(final String[] args) {
    for (final Stream<Character> p : perms(fromString(args[0]).toStream())) {
      System.out.println(asString(p.toList()));
    }
  }

  public static Stream<Stream<Character>> perms(final Stream<Character> s) {
    Stream<Stream<Character>> r = single(Stream.<Character>nil());
    for (final Zipper<Character> z : fromStream(s))
      r = join(z.cobind(new F<Zipper<Character>, Stream<Stream<Character>>>() {
        public Stream<Stream<Character>> f(final Zipper<Character> zp) {
          return perms(zp.lefts().reverse().append(zp.rights()))
              .map(Stream.<Character>cons().f(zp.focus()).o(P.<Stream<Character>>p1()));
        }
      }).toStream());
    return r;
  }
}

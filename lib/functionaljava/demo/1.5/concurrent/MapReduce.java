package concurrent;

import fj.F;
import fj.P1;
import fj.Unit;
import fj.control.parallel.ParModule;
import static fj.control.parallel.ParModule.parModule;
import fj.control.parallel.Promise;
import fj.control.parallel.Strategy;
import static fj.data.LazyString.fromStream;
import fj.data.List;
import static fj.data.List.list;
import fj.data.Option;
import static fj.data.Option.fromNull;
import fj.data.Stream;
import static fj.data.Stream.fromString;
import static fj.Monoid.longAdditionMonoid;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.concurrent.ExecutorService;
import static java.util.concurrent.Executors.newFixedThreadPool;

/* Performs a parallel word count over files given as program arguments. */
public class MapReduce {

  // Count words of documents in parallel
  public static Promise<Long> countWords(final List<Stream<Character>> documents,
                                         final ParModule m) {
    return m.parFoldMap(documents, new F<Stream<Character>, Long>() {
      public Long f(final Stream<Character> document) {
        return (long) fromStream(document).words().length();
      }
    }, longAdditionMonoid);
  }

  // Main program does the requisite IO gymnastics
  public static void main(final String[] args) {
    final List<Stream<Character>> documents = list(args).map(
        new F<String, BufferedReader>() {
          public BufferedReader f(final String fileName) {
            try {
              return new BufferedReader(new FileReader(new File(fileName)));
            } catch (FileNotFoundException e) {
              throw new Error(e);
            }
          }
        }.andThen(new F<BufferedReader, Stream<Character>>() {
          public Stream<Character> f(final BufferedReader reader) {
            final Option<String> s;
            try {
              s = fromNull(reader.readLine());
            } catch (IOException e) {
              throw new Error(e);
            }
            if (s.isSome())
              return fromString(s.some()).append(new P1<Stream<Character>>() {
                public Stream<Character> _1() {
                  return f(reader);
                }
              });
            else {
              try {
                reader.close();
              } catch (IOException e) {
                throw new Error(e);
              }
              return Stream.nil();
            }
          }
        }));

    final ExecutorService pool = newFixedThreadPool(16);
    final ParModule m = parModule(Strategy.<Unit>executorStrategy(pool));

    System.out.println("Word Count: " + countWords(documents, m).claim());

    pool.shutdown();
  }
}